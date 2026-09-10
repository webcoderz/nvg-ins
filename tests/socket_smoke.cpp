#include <arpa/inet.h>
#include <cassert>
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
  assert(argc == 2);
  const int descriptor = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
  assert(descriptor >= 0);
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  assert(std::strlen(argv[1]) < sizeof(address.sun_path));
  std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", argv[1]);
  assert(::connect(descriptor, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);

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
  assert(pb_encode(&output, nvg_ins_v1_Envelope_fields, &envelope));
  std::uint32_t length = htonl(static_cast<std::uint32_t>(output.bytes_written));
  assert(transfer(descriptor, &length, sizeof(length), true));
  assert(transfer(descriptor, payload, output.bytes_written, true));

  assert(transfer(descriptor, &length, sizeof(length), false));
  const auto response_size = ntohl(length);
  assert(response_size > 0 && response_size <= sizeof(payload));
  assert(transfer(descriptor, payload, response_size, false));
  nvg_ins_v1_Envelope response = nvg_ins_v1_Envelope_init_zero;
  auto input = pb_istream_from_buffer(payload, response_size);
  assert(pb_decode(&input, nvg_ins_v1_Envelope_fields, &response));
  assert(response.which_payload == nvg_ins_v1_Envelope_state_tag);
  assert(response.payload.state.sequence == 1);
  assert(response.payload.state.mapped_capture_monotonic_ns ==
         request.mapped_capture_monotonic_ns);
  assert(std::string(response.payload.state.calibration_hash) ==
         request.calibration_hash);
  ::close(descriptor);
  return 0;
}
