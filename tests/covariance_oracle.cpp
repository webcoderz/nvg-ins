#include "estimator.hpp"
#include "ins_capi.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

namespace {

template <std::size_t Size>
void text(char (&destination)[Size], const char* value) {
  std::snprintf(destination, Size, "%s", value);
}

nvg_ins_v1_EstimatorRequest request(std::uint64_t sequence) {
  nvg_ins_v1_EstimatorRequest value = nvg_ins_v1_EstimatorRequest_init_zero;
  value.protocol_version = 1;
  value.sequence = sequence;
  std::snprintf(value.request_id, sizeof(value.request_id), "covariance-oracle-%llu",
                static_cast<unsigned long long>(sequence));
  text(value.estimator_session_id, "estimator-1");
  text(value.clock_epoch, "clock-1");
  text(value.origin_epoch, "origin-1");
  text(value.calibration_hash,
       "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
  value.mapped_capture_monotonic_ns = sequence * 640'000'000;
  value.mapped_capture_unix_ns =
      1'700'000'000'000'000'000ULL + value.mapped_capture_monotonic_ns;
  value.timing_uncertainty_ns = 1'000;
  value.imu_samples_count = 64;
  for (pb_size_t index = 0; index < value.imu_samples_count; ++index) {
    auto& sample = value.imu_samples[index];
    sample.capture_time_us = ((sequence - 1) * 64 + index + 1) * 10'000;
    sample.has_specific_force_frd_mps2 = true;
    sample.specific_force_frd_mps2.z = -9.80665;
    sample.has_angular_rate_frd_rad_s = true;
    sample.accelerometer_valid = true;
    sample.gyroscope_valid = true;
    text(sample.calibration_id, "imu-calibration-1");
  }
  value.has_gnss = true;
  value.gnss.capture_time_us = sequence * 640'000;
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

  void* oracle = ins_suite_create();
  assert(oracle != nullptr);
  ins_cfg_t config{};
  config.time_us = 10'000;
  config.auto_init = 1;
  assert(ins_suite_init(oracle, &config) == 0);

  std::uint64_t previous_time_us = 0;
  for (std::uint64_t sequence = 1; sequence <= 10; ++sequence) {
    const auto input = request(sequence);
    assert(estimator.process(input, state, error));
    for (pb_size_t index = 0; index < input.imu_samples_count; ++index) {
      const auto& sample = input.imu_samples[index];
      const auto dt = previous_time_us == 0
                          ? 0.0F
                          : static_cast<float>(sample.capture_time_us - previous_time_us) /
                                1'000'000.0F;
      previous_time_us = sample.capture_time_us;
      const float acceleration[3]{
          static_cast<float>(sample.specific_force_frd_mps2.x),
          static_cast<float>(sample.specific_force_frd_mps2.y),
          static_cast<float>(sample.specific_force_frd_mps2.z)};
      const float angular_rate[3]{
          static_cast<float>(sample.angular_rate_frd_rad_s.x),
          static_cast<float>(sample.angular_rate_frd_rad_s.y),
          static_cast<float>(sample.angular_rate_frd_rad_s.z)};
      const float acceleration_variance[3]{};
      const float angular_rate_variance[3]{};
      ins_suite_set_imu(oracle, static_cast<std::int64_t>(sample.capture_time_us), dt,
                        acceleration, angular_rate, acceleration_variance,
                        angular_rate_variance);
      if (index + 1 == input.imu_samples_count) {
        const double ecef[3]{input.gnss.position_ecef_m.x,
                             input.gnss.position_ecef_m.y,
                             input.gnss.position_ecef_m.z};
        const float velocity_ned[3]{
            static_cast<float>(input.gnss.velocity_ned_mps.x),
            static_cast<float>(input.gnss.velocity_ned_mps.y),
            static_cast<float>(input.gnss.velocity_ned_mps.z)};
        float position_covariance[9]{};
        float velocity_covariance[9]{};
        std::transform(std::begin(input.gnss.position_covariance_m2),
                       std::end(input.gnss.position_covariance_m2),
                       position_covariance,
                       [](double value) { return static_cast<float>(value); });
        std::transform(std::begin(input.gnss.velocity_covariance_m2ps2),
                       std::end(input.gnss.velocity_covariance_m2ps2),
                       velocity_covariance,
                       [](double value) { return static_cast<float>(value); });
        ins_suite_set_gnss_pos_ecef_cov(oracle, ecef, position_covariance);
        ins_suite_set_gnss_vel_ned_cov(oracle, velocity_ned, velocity_covariance);
      }
      ins_suite_update(oracle);
    }
  }

  float expected[INS_CAPI_MAX_STATE * INS_CAPI_MAX_STATE]{};
  const int dimension = ins_suite_get_covariance(oracle, expected);
  assert(dimension == static_cast<int>(state.covariance_order_count));
  assert(state.covariance_count == static_cast<pb_size_t>(dimension * dimension));
  for (int row = 0; row < dimension; ++row) {
    for (int column = 0; column < dimension; ++column) {
      const double actual = state.covariance[row * dimension + column];
      const double oracle_value = expected[row + column * INS_CAPI_MAX_STATE];
      const double tolerance =
          1e-6 * std::max({1.0, std::abs(actual), std::abs(oracle_value)});
      assert(std::abs(actual - oracle_value) <= tolerance);
    }
  }
  ins_suite_destroy(oracle);
  return 0;
}
