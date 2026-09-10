#include "estimator.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <sched.h>
#include <string>
#include <time.h>

#include <pb_decode.h>
#include <pb_encode.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

constexpr std::size_t kMaximumPayload = 64 * 1024;
volatile std::sig_atomic_t stop_requested = 0;

struct Options {
  std::string socket_path{"/run/nvg/ins.sock"};
  std::string replay_path;
  std::string output_path;
  unsigned output_hz{100};
  int cpu{-1};
  int policy{SCHED_OTHER};
  int priority{0};
};

void stop(int) { stop_requested = 1; }

bool parse_integer(const char* text, int minimum, int maximum, int& output) {
  char* end = nullptr;
  errno = 0;
  const long value = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || value < minimum || value > maximum) {
    return false;
  }
  output = static_cast<int>(value);
  return true;
}

bool parse_options(int argc, char** argv, Options& options) {
  for (int index = 1; index < argc; ++index) {
    const std::string key = argv[index];
    if (index + 1 >= argc) return false;
    const char* value = argv[++index];
    if (key == "--socket") {
      options.socket_path = value;
    } else if (key == "--replay") {
      options.replay_path = value;
    } else if (key == "--output") {
      options.output_path = value;
    } else if (key == "--output-hz") {
      int parsed = 0;
      if (!parse_integer(value, 1, 1000, parsed)) return false;
      options.output_hz = static_cast<unsigned>(parsed);
    } else if (key == "--cpu") {
      if (!parse_integer(value, 0, CPU_SETSIZE - 1, options.cpu)) return false;
    } else if (key == "--sched") {
      if (std::strcmp(value, "other") == 0) options.policy = SCHED_OTHER;
      else if (std::strcmp(value, "fifo") == 0) options.policy = SCHED_FIFO;
      else return false;
    } else if (key == "--priority") {
      if (!parse_integer(value, 0, 99, options.priority)) return false;
    } else {
      return false;
    }
  }
  const bool replay = !options.replay_path.empty() || !options.output_path.empty();
  if (replay != (!options.replay_path.empty() && !options.output_path.empty())) return false;
  if (options.policy == SCHED_OTHER && options.priority != 0) return false;
  if (options.policy == SCHED_FIFO && options.priority == 0) return false;
  return options.socket_path.size() < sizeof(sockaddr_un::sun_path);
}

bool configure_scheduler(const Options& options) {
  if (options.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(options.cpu, &set);
    if (::sched_setaffinity(0, sizeof(set), &set) != 0) {
      std::perror("sched_setaffinity");
      return false;
    }
  }
  sched_param parameters{};
  parameters.sched_priority = options.priority;
  if (::sched_setscheduler(0, options.policy, &parameters) != 0) {
    std::perror("sched_setscheduler");
    return false;
  }
  return true;
}

bool read_exact(int descriptor, void* output, std::size_t length, bool& clean_eof) {
  auto* cursor = static_cast<std::uint8_t*>(output);
  clean_eof = false;
  while (length > 0) {
    const auto received = ::read(descriptor, cursor, length);
    if (received == 0) {
      clean_eof = cursor == output;
      return false;
    }
    if (received < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    cursor += received;
    length -= static_cast<std::size_t>(received);
  }
  return true;
}

bool write_exact(int descriptor, const void* input, std::size_t length) {
  const auto* cursor = static_cast<const std::uint8_t*>(input);
  while (length > 0) {
    const auto written = ::write(descriptor, cursor, length);
    if (written < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    cursor += written;
    length -= static_cast<std::size_t>(written);
  }
  return true;
}

bool authorized_peer(int descriptor) {
#ifdef SO_PEERCRED
  struct ucred credentials {};
  socklen_t size = sizeof(credentials);
  return ::getsockopt(descriptor, SOL_SOCKET, SO_PEERCRED, &credentials, &size) == 0 &&
         credentials.uid == ::geteuid();
#else
  (void)descriptor;
  return false;
#endif
}

std::uint64_t monotonic_ns() {
  timespec value{};
  ::clock_gettime(CLOCK_MONOTONIC, &value);
  return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL + value.tv_nsec;
}

void pace(std::uint64_t period_ns, std::uint64_t& previous_output_ns,
          nvg_ins_v1_EstimatorState& state) {
  auto now = monotonic_ns();
  if (previous_output_ns != 0 && now - previous_output_ns < period_ns) {
    const auto remaining = period_ns - (now - previous_output_ns);
    timespec delay{static_cast<time_t>(remaining / 1'000'000'000ULL),
                   static_cast<long>(remaining % 1'000'000'000ULL)};
    while (::nanosleep(&delay, &delay) != 0 && errno == EINTR && !stop_requested) {}
    now = monotonic_ns();
  }
  state.actual_output_hz = previous_output_ns == 0 || now <= previous_output_ns
      ? 0
      : static_cast<std::uint32_t>(std::min<std::uint64_t>(
            1000, 1'000'000'000ULL / (now - previous_output_ns)));
  previous_output_ns = now;
}

bool process_stream(int input_descriptor, int output_descriptor, unsigned output_hz,
                    bool realtime) {
  nvg::ins::Estimator estimator;
  std::uint8_t payload[kMaximumPayload]{};
  std::uint8_t encoded[kMaximumPayload]{};
  std::uint64_t previous_output_ns = 0;
  const std::uint64_t period_ns = 1'000'000'000ULL / output_hz;
  while (!stop_requested) {
    std::uint32_t network_length = 0;
    bool clean_eof = false;
    if (!read_exact(input_descriptor, &network_length, sizeof(network_length), clean_eof)) {
      return clean_eof;
    }
    const auto length = static_cast<std::size_t>(ntohl(network_length));
    if (length == 0 || length > kMaximumPayload ||
        !read_exact(input_descriptor, payload, length, clean_eof)) {
      return false;
    }
    nvg_ins_v1_Envelope request_envelope = nvg_ins_v1_Envelope_init_zero;
    auto input = pb_istream_from_buffer(payload, length);
    if (!pb_decode(&input, nvg_ins_v1_Envelope_fields, &request_envelope) ||
        request_envelope.which_payload != nvg_ins_v1_Envelope_request_tag) {
      return false;
    }
    nvg_ins_v1_EstimatorState state = nvg_ins_v1_EstimatorState_init_zero;
    std::string error;
    if (!estimator.process(request_envelope.payload.request, state, error)) {
      std::fprintf(stderr, "nvg-ins rejected request: %s\n", error.c_str());
      return false;
    }
    if (realtime) pace(period_ns, previous_output_ns, state);
    else state.actual_output_hz = 0;
    nvg_ins_v1_Envelope response = nvg_ins_v1_Envelope_init_zero;
    response.which_payload = nvg_ins_v1_Envelope_state_tag;
    response.payload.state = state;
    auto output = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!pb_encode(&output, nvg_ins_v1_Envelope_fields, &response)) return false;
    network_length = htonl(static_cast<std::uint32_t>(output.bytes_written));
    if (!write_exact(output_descriptor, &network_length, sizeof(network_length)) ||
        !write_exact(output_descriptor, encoded, output.bytes_written)) {
      return false;
    }
  }
  return true;
}

int replay(const Options& options) {
  const int input = ::open(options.replay_path.c_str(), O_RDONLY | O_CLOEXEC);
  if (input < 0) {
    std::perror("open replay input");
    return 1;
  }
  const int output = ::open(options.output_path.c_str(),
                            O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
  if (output < 0) {
    std::perror("open replay output");
    ::close(input);
    return 1;
  }
  const bool success = process_stream(input, output, options.output_hz, false);
  ::close(output);
  ::close(input);
  return success ? 0 : 1;
}

int serve(const Options& options) {
  ::umask(0077);
  const int listener = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listener < 0) {
    std::perror("socket");
    return 1;
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", options.socket_path.c_str());
  ::unlink(address.sun_path);
  if (::bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
      ::chmod(address.sun_path, 0600) != 0 || ::listen(listener, 4) != 0) {
    std::perror("bind/listen");
    ::close(listener);
    ::unlink(address.sun_path);
    return 1;
  }
  std::printf("nvg-ins ready socket=%s protocol=1 max_payload=%zu output_hz=%u\n",
              options.socket_path.c_str(), kMaximumPayload, options.output_hz);
  std::fflush(stdout);
  while (!stop_requested) {
    const int client = ::accept4(listener, nullptr, nullptr, SOCK_CLOEXEC);
    if (client < 0) {
      if (errno == EINTR) continue;
      std::perror("accept4");
      break;
    }
    if (!authorized_peer(client)) {
      std::fprintf(stderr, "nvg-ins rejected foreign peer\n");
      ::close(client);
      continue;
    }
    if (!process_stream(client, client, options.output_hz, true)) {
      std::fprintf(stderr, "nvg-ins closed malformed client\n");
    }
    ::close(client);
  }
  ::close(listener);
  ::unlink(address.sun_path);
  return stop_requested ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse_options(argc, argv, options)) {
    std::fprintf(stderr,
                 "usage: nvg-ins [--socket PATH] [--output-hz 1..1000] [--cpu N] "
                 "[--sched other|fifo] [--priority N]\n"
                 "       nvg-ins --replay FILE --output FILE [scheduling options]\n");
    return 2;
  }
  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  if (!configure_scheduler(options)) return 1;
  return options.replay_path.empty() ? serve(options) : replay(options);
}
