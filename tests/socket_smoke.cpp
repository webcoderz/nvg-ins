#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include <pb_decode.h>
#include <pb_encode.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "ins.pb.h"

// `assert` disappears under NDEBUG, which is exactly how these tests are built (Release), taking
// the checks with it and leaving the variables they used unused -- which -Werror then refuses to
// compile. A test that vanishes in the build everyone ships is not a test, so this one keeps its
// checks in every configuration.
#define CHECK(condition)                                                          \
  do {                                                                            \
    if (!(condition)) {                                                           \
      std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__,       \
                   #condition);                                                   \
      std::abort();                                                               \
    }                                                                             \
  } while (0)


namespace {

template <std::size_t Size>
void text(char (&destination)[Size], const char* value) {
  std::snprintf(destination, Size, "%s", value);
}

bool transfer(int descriptor, void* bytes, std::size_t size, bool send) {
  auto* cursor = static_cast<std::uint8_t*>(bytes);
  while (size > 0) {
    const auto count = send ? ::write(descriptor, cursor, size)
                            : ::read(descriptor, cursor, size);
    if (count <= 0) return false;
    cursor += count;
    size -= static_cast<std::size_t>(count);
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  CHECK(argc == 2);
  const int descriptor = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  CHECK(descriptor >= 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  CHECK(std::strlen(argv[1]) < sizeof(address.sun_path));
  std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", argv[1]);
  CHECK(::connect(descriptor, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);

  nvg_ins_v1_Envelope envelope = nvg_ins_v1_Envelope_init_zero;
  envelope.which_payload = nvg_ins_v1_Envelope_request_tag;
  auto& request = envelope.payload.request;
  request.protocol_version = 1;
  request.sequence = 1;
  text(request.request_id, "socket-smoke-1");
  text(request.estimator_session_id, "socket-smoke");
  text(request.clock_epoch, "clock-1");
  text(request.origin_epoch, "origin-1");
  text(request.calibration_hash,
       "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
  request.mapped_capture_monotonic_ns = 10'000'000;
  request.mapped_capture_unix_ns = 1'700'000'000'010'000'000ULL;
  request.timing_uncertainty_ns = 1'000;
  request.imu_samples_count = 1;
  auto& imu = request.imu_samples[0];
  imu.capture_time_us = 10'000;
  imu.has_specific_force_frd_mps2 = true;
  imu.specific_force_frd_mps2.z = -9.80665;
  imu.has_angular_rate_frd_rad_s = true;
  imu.accelerometer_valid = true;
  imu.gyroscope_valid = true;
  text(imu.calibration_id, "imu-calibration-1");

  std::uint8_t payload[64 * 1024]{};
  auto output = pb_ostream_from_buffer(payload, sizeof(payload));
  CHECK(pb_encode(&output, nvg_ins_v1_Envelope_fields, &envelope));
  std::uint32_t length = htonl(static_cast<std::uint32_t>(output.bytes_written));
  CHECK(transfer(descriptor, &length, sizeof(length), true));
  CHECK(transfer(descriptor, payload, output.bytes_written, true));

  CHECK(transfer(descriptor, &length, sizeof(length), false));
  const auto response_size = ntohl(length);
  CHECK(response_size > 0 && response_size <= sizeof(payload));
  CHECK(transfer(descriptor, payload, response_size, false));
  nvg_ins_v1_Envelope response = nvg_ins_v1_Envelope_init_zero;
  auto input = pb_istream_from_buffer(payload, response_size);
  CHECK(pb_decode(&input, nvg_ins_v1_Envelope_fields, &response));
  CHECK(response.which_payload == nvg_ins_v1_Envelope_state_tag);
  CHECK(response.payload.state.sequence == 1);
  CHECK(response.payload.state.mapped_capture_monotonic_ns ==
         request.mapped_capture_monotonic_ns);
  CHECK(std::string(response.payload.state.calibration_hash) ==
         request.calibration_hash);
  ::close(descriptor);
  return 0;
}
