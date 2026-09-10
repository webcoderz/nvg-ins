#include "estimator.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <pb_decode.h>
#include <pb_encode.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

constexpr std::size_t kMaximumPayload = 64 * 1024;
volatile std::sig_atomic_t stop_requested = 0;

void stop(int) { stop_requested = 1; }

bool read_exact(int descriptor, void* output, std::size_t length) {
  auto* cursor = static_cast<std::uint8_t*>(output);
  while (length > 0) {
    const auto received = ::recv(descriptor, cursor, length, 0);
    if (received == 0) return false;
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
    const auto written = ::send(descriptor, cursor, length, MSG_NOSIGNAL);
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

bool serve_client(int descriptor) {
  nvg::ins::Estimator estimator;
  std::uint8_t payload[kMaximumPayload]{};
  std::uint8_t encoded[kMaximumPayload]{};
  while (!stop_requested) {
    std::uint32_t network_length = 0;
    if (!read_exact(descriptor, &network_length, sizeof(network_length))) return true;
    const auto length = static_cast<std::size_t>(ntohl(network_length));
    if (length == 0 || length > kMaximumPayload ||
        !read_exact(descriptor, payload, length)) {
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
    nvg_ins_v1_Envelope response = nvg_ins_v1_Envelope_init_zero;
    response.which_payload = nvg_ins_v1_Envelope_state_tag;
    response.payload.state = state;
    auto output = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!pb_encode(&output, nvg_ins_v1_Envelope_fields, &response) ||
        output.bytes_written > kMaximumPayload) {
      return false;
    }
    network_length = htonl(static_cast<std::uint32_t>(output.bytes_written));
    if (!write_exact(descriptor, &network_length, sizeof(network_length)) ||
        !write_exact(descriptor, encoded, output.bytes_written)) {
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  std::string socket_path = "/run/nvg/ins.sock";
  if (argc == 3 && std::strcmp(argv[1], "--socket") == 0) {
    socket_path = argv[2];
  } else if (argc != 1) {
    std::fprintf(stderr, "usage: nvg-ins [--socket PATH]\n");
    return 2;
  }
  if (socket_path.empty() || socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
    std::fprintf(stderr, "nvg-ins socket path is invalid\n");
    return 2;
  }

  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  ::umask(0077);
  const int listener = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (listener < 0) {
    std::perror("socket");
    return 1;
  }
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket_path.c_str());
  ::unlink(address.sun_path);
  if (::bind(listener, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
      ::chmod(address.sun_path, 0600) != 0 || ::listen(listener, 4) != 0) {
    std::perror("bind/listen");
    ::close(listener);
    ::unlink(address.sun_path);
    return 1;
  }
  std::printf("nvg-ins ready socket=%s protocol=1 max_payload=%zu\n",
              socket_path.c_str(), kMaximumPayload);
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
    if (!serve_client(client)) {
      std::fprintf(stderr, "nvg-ins closed malformed client\n");
    }
    ::close(client);
  }
  ::close(listener);
  ::unlink(address.sun_path);
  return stop_requested ? 0 : 1;
}
