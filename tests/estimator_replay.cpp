#include "estimator.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstdio>
#include <string>

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

nvg_ins_v1_EstimatorRequest request(std::uint64_t sequence, const char* clock_epoch) {
  nvg_ins_v1_EstimatorRequest value = nvg_ins_v1_EstimatorRequest_init_zero;
  value.protocol_version = 1;
  value.sequence = sequence;
  text(value.request_id, sequence == 1 ? "request-1" : "request-2");
  text(value.estimator_session_id, "estimator-1");
  text(value.clock_epoch, clock_epoch);
  text(value.origin_epoch, "origin-1");
  text(value.calibration_hash,
       "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
  value.mapped_capture_monotonic_ns = 640'000'000;
  value.mapped_capture_unix_ns = 1'700'000'000'640'000'000ULL;
  value.timing_uncertainty_ns = 1'000;
  value.imu_samples_count = 64;
  for (pb_size_t index = 0; index < value.imu_samples_count; ++index) {
    auto& sample = value.imu_samples[index];
    sample.capture_time_us = static_cast<std::uint64_t>(index + 1) * 10'000;
    sample.has_specific_force_frd_mps2 = true;
    sample.specific_force_frd_mps2.z = -9.80665;
    sample.has_angular_rate_frd_rad_s = true;
    sample.accelerometer_valid = true;
    sample.gyroscope_valid = true;
    text(sample.calibration_id, "imu-calibration-1");
  }
  value.has_gnss = true;
  value.gnss.capture_time_us = 640'000;
  value.gnss.has_position_ecef_m = true;
  value.gnss.position_ecef_m.x = 6'378'137.0;
  value.gnss.has_velocity_ned_mps = true;
  value.gnss.position_valid = true;
  value.gnss.velocity_valid = true;
  text(value.gnss.calibration_id, "gnss-calibration-1");
  for (std::size_t axis = 0; axis < 3; ++axis) {
    value.gnss.position_covariance_m2[axis * 3 + axis] = 1.0;
    value.gnss.velocity_covariance_m2ps2[axis * 3 + axis] = 0.01;
  }
  return value;
}

}  // namespace

int main() {
  nvg::ins::Estimator estimator;
  nvg_ins_v1_EstimatorState state = nvg_ins_v1_EstimatorState_init_zero;
  std::string error;

  auto first = request(1, "clock-1");
  CHECK(estimator.process(first, state, error));
  CHECK(state.protocol_version == 1);
  CHECK(state.sequence == 1);
  CHECK(state.mapped_capture_monotonic_ns == first.mapped_capture_monotonic_ns);
  CHECK(state.timing_uncertainty_ns == first.timing_uncertainty_ns);
  CHECK(state.covariance_order_count == 15);
  CHECK(state.covariance_count == 225);
  CHECK(state.aiding_mode == nvg_ins_v1_AidingMode_AIDING_MODE_LOOSE);

  error.clear();
  CHECK(!estimator.process(first, state, error));
  CHECK(error == "request sequence is not strictly increasing");

  auto reset = request(1, "clock-2");
  error.clear();
  CHECK(estimator.process(reset, state, error));
  CHECK(std::string(state.clock_epoch) == "clock-2");
  return 0;
}
