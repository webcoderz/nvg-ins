#include "estimator.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace {

constexpr std::uint32_t kProtocolVersion = 1;
constexpr std::uint64_t kMaximumTimingUncertaintyNs = 100'000'000;

bool finite_value(double value) { return std::isfinite(value); }

bool valid_hash(const char* value) {
  if (std::strlen(value) != 64) return false;
  return std::all_of(value, value + 64, [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}

bool valid_identifier(const char* value) {
  const auto length = std::strlen(value);
  if (length == 0 || length > 64) return false;
  return std::all_of(value, value + length, [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' ||
           c == ':' || c == '-';
  });
}

bool valid_vector(const nvg_ins_v1_Vector3& value) {
  return finite_value(value.x) && finite_value(value.y) && finite_value(value.z);
}

bool valid_covariance(const double* values, std::size_t dimension) {
  constexpr double kTolerance = 1e-9;
  std::array<double, 16> factor{};
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t column = 0; column < dimension; ++column) {
      const auto value = values[row * dimension + column];
      if (!finite_value(value) ||
          std::abs(value - values[column * dimension + row]) > kTolerance) {
        return false;
      }
    }
  }
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t column = 0; column <= row; ++column) {
      double value = values[row * dimension + column];
      for (std::size_t k = 0; k < column; ++k) {
        value -= factor[row * dimension + k] * factor[column * dimension + k];
      }
      if (row == column) {
        if (value < -kTolerance) return false;
        factor[row * dimension + column] = value > kTolerance ? std::sqrt(value) : 0.0;
      } else if (factor[column * dimension + column] > kTolerance) {
        factor[row * dimension + column] = value / factor[column * dimension + column];
      } else if (std::abs(value) > kTolerance) {
        return false;
      }
    }
  }
  return true;
}

template <std::size_t Size>
void copy_text(char (&destination)[Size], const char* source) {
  std::snprintf(destination, Size, "%s", source);
}

void copy_vector(const nvg_ins_v1_Vector3& source, float destination[3]) {
  destination[0] = static_cast<float>(source.x);
  destination[1] = static_cast<float>(source.y);
  destination[2] = static_cast<float>(source.z);
}

void set_vector(nvg_ins_v1_Vector3& destination, const float source[3]) {
  destination.x = source[0];
  destination.y = source[1];
  destination.z = source[2];
}

void rpy_to_quaternion(float roll, float pitch, float yaw,
                       nvg_ins_v1_QuaternionWxyz& quaternion) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  quaternion.w = cr * cp * cy + sr * sp * sy;
  quaternion.x = sr * cp * cy - cr * sp * sy;
  quaternion.y = cr * sp * cy + sr * cp * sy;
  quaternion.z = cr * cp * sy - sr * sp * cy;
}

constexpr std::array<const char*, 18> kCovarianceOrder{
    "position_n", "position_e", "position_d", "velocity_n", "velocity_e",
    "velocity_d", "attitude_roll", "attitude_pitch", "attitude_yaw",
    "accelerometer_bias_x", "accelerometer_bias_y", "accelerometer_bias_z",
    "gyro_bias_x", "gyro_bias_y", "gyro_bias_z", "magnetometer_bias_x",
    "magnetometer_bias_y", "magnetometer_bias_z"};

}  // namespace

namespace nvg::ins {

Estimator::Estimator() = default;

bool Estimator::validate(const nvg_ins_v1_EstimatorRequest& request,
                         std::string& error) const {
  if (request.protocol_version != kProtocolVersion ||
      !valid_identifier(request.request_id) ||
      !valid_identifier(request.estimator_session_id) ||
      !valid_identifier(request.clock_epoch) ||
      !valid_identifier(request.origin_epoch)) {
    error = "invalid protocol version or identity";
    return false;
  }
  if (request.mapped_capture_monotonic_ns == 0 || request.mapped_capture_unix_ns == 0 ||
      request.timing_uncertainty_ns > kMaximumTimingUncertaintyNs ||
      !valid_hash(request.calibration_hash)) {
    error = "invalid capture mapping or calibration hash";
    return false;
  }
  std::uint64_t previous = 0;
  for (pb_size_t index = 0; index < request.imu_samples_count; ++index) {
    const auto& sample = request.imu_samples[index];
    if (sample.capture_time_us <= previous || !valid_vector(sample.specific_force_frd_mps2) ||
        !valid_vector(sample.angular_rate_frd_rad_s) ||
        !valid_identifier(sample.calibration_id)) {
      error = "invalid or unordered IMU sample";
      return false;
    }
    previous = sample.capture_time_us;
  }
  if (request.has_gnss &&
      (!valid_identifier(request.gnss.calibration_id) ||
       !valid_vector(request.gnss.position_ecef_m) ||
       !valid_vector(request.gnss.velocity_ned_mps) ||
       !valid_covariance(request.gnss.position_covariance_m2, 3) ||
       !valid_covariance(request.gnss.velocity_covariance_m2ps2, 3))) {
    error = "invalid GNSS aiding";
    return false;
  }
  if (request.has_pressure &&
      (!valid_identifier(request.pressure.calibration_id) ||
       !finite_value(request.pressure.pressure_pa) || request.pressure.pressure_pa < 1000.0 ||
       request.pressure.pressure_pa > 120000.0 ||
       !finite_value(request.pressure.altitude_sigma_m) || request.pressure.altitude_sigma_m < 0.0)) {
    error = "invalid pressure aiding";
    return false;
  }
  if (request.has_magnetometer &&
      (!valid_identifier(request.magnetometer.calibration_id) ||
       !valid_vector(request.magnetometer.magnetic_field_frd_ut) ||
       !valid_covariance(request.magnetometer.covariance_ut2, 3))) {
    error = "invalid magnetometer aiding";
    return false;
  }
  if (request.has_vio &&
      (!valid_identifier(request.vio.calibration_id) ||
       !valid_identifier(request.vio.estimator_epoch) ||
       !valid_vector(request.vio.position_ned_m) || !finite_value(request.vio.yaw_rad) ||
       (request.vio.covariance_count != 9 && request.vio.covariance_count != 16) ||
       !valid_covariance(request.vio.covariance,
                         request.vio.covariance_count == 9 ? 3 : 4))) {
    error = "invalid VIO aiding";
    return false;
  }
  return true;
}

bool Estimator::reset(const nvg_ins_v1_EstimatorRequest& request, std::string& error) {
  suite_ = {};
  ins_init_t init{};
  ins_options_t options{};
  init.time = request.imu_samples_count > 0
                  ? static_cast<ins_time_us_t>(request.imu_samples[0].capture_time_us)
                  : 1;
  init.x_ecef[0] = request.has_gnss && request.gnss.position_valid
                       ? request.gnss.position_ecef_m.x
                       : 6'378'137.0;
  init.x_ecef[1] = request.has_gnss && request.gnss.position_valid
                       ? request.gnss.position_ecef_m.y
                       : 0.0;
  init.x_ecef[2] = request.has_gnss && request.gnss.position_valid
                       ? request.gnss.position_ecef_m.z
                       : 0.0;
  options.auto_init = true;
  if (nav_suite_init(&suite_, &init, &options) != 0) {
    error = "INSLIB initialization failed";
    return false;
  }
  session_ = request.estimator_session_id;
  clock_epoch_ = request.clock_epoch;
  origin_epoch_ = request.origin_epoch;
  last_sequence_ = 0;
  last_imu_time_us_ = 0;
  initialized_ = true;
  return true;
}

void Estimator::apply(const nvg_ins_v1_EstimatorRequest& request) {
  const auto updates = std::max<pb_size_t>(request.imu_samples_count, 1);
  for (pb_size_t index = 0; index < updates; ++index) {
    const bool final = index + 1 == updates;
    ins_measurements_t measurement{};
    if (request.imu_samples_count > 0) {
      const auto& sample = request.imu_samples[index];
      measurement.timestamp = static_cast<ins_time_us_t>(sample.capture_time_us);
      measurement.strapdown_dt_sec = last_imu_time_us_ > 0
          ? static_cast<float>(measurement.timestamp - last_imu_time_us_) / 1'000'000.0F
          : 0.0F;
      measurement.acc.is_valid = sample.accelerometer_valid && !sample.accelerometer_saturated;
      measurement.gyr.is_valid = sample.gyroscope_valid && !sample.gyroscope_saturated;
      copy_vector(sample.specific_force_frd_mps2, measurement.acc.data);
      copy_vector(sample.angular_rate_frd_rad_s, measurement.gyr.data);
      last_imu_time_us_ = measurement.timestamp;
    } else {
      measurement.timestamp = static_cast<ins_time_us_t>(
          request.mapped_capture_monotonic_ns / 1000);
    }
    if (final && request.has_gnss) {
      const auto& gnss = request.gnss;
      measurement.gnss_pos.is_valid = gnss.position_valid;
      measurement.gnss_vel.is_valid = gnss.velocity_valid;
      measurement.gnss_pos.xyz_ecef[0] = gnss.position_ecef_m.x;
      measurement.gnss_pos.xyz_ecef[1] = gnss.position_ecef_m.y;
      measurement.gnss_pos.xyz_ecef[2] = gnss.position_ecef_m.z;
      copy_vector(gnss.velocity_ned_mps, measurement.gnss_vel.vel_ned);
      for (std::size_t item = 0; item < 9; ++item) {
        measurement.gnss_pos.Qll_ned[item] = static_cast<float>(gnss.position_covariance_m2[item]);
        measurement.gnss_vel.Qll_ned[item] = static_cast<float>(gnss.velocity_covariance_m2ps2[item]);
      }
    }
    if (final && request.has_pressure && request.pressure.valid) {
      measurement.baro.is_valid = true;
      measurement.baro.pressure_pa = static_cast<float>(request.pressure.pressure_pa);
      measurement.baro.stddev_m = static_cast<float>(request.pressure.altitude_sigma_m);
    }
    if (final && request.has_magnetometer && request.magnetometer.valid) {
      measurement.mag.is_valid = true;
      copy_vector(request.magnetometer.magnetic_field_frd_ut, measurement.mag.data);
      for (std::size_t axis = 0; axis < 3; ++axis) {
        measurement.mag.Qll_diag[axis] =
            static_cast<float>(request.magnetometer.covariance_ut2[axis * 3 + axis]);
      }
    }
    if (final && request.has_vio && request.vio.valid) {
      measurement.local_pos.is_valid = true;
      copy_vector(request.vio.position_ned_m, measurement.local_pos.pos_ned);
      for (std::size_t item = 0; item < 9; ++item) {
        measurement.local_pos.Qll_ned[item] = static_cast<float>(request.vio.covariance[item]);
      }
      measurement.yaw.is_valid = true;
      measurement.yaw.yaw_rad = static_cast<float>(request.vio.yaw_rad);
      const auto dimension = request.vio.covariance_count == 16 ? 4U : 3U;
      measurement.yaw.stddev_rad = static_cast<float>(
          std::sqrt(request.vio.covariance[(dimension - 1) * dimension + dimension - 1]));
    }
    if (final && request.has_yaw_reference && request.yaw_reference.valid) {
      measurement.yaw.is_valid = true;
      measurement.yaw.yaw_rad = static_cast<float>(request.yaw_reference.yaw_rad);
      measurement.yaw.stddev_rad = static_cast<float>(request.yaw_reference.yaw_sigma_rad);
    }
    if (final && request.has_zero_motion) {
      measurement.zero_velocity_update = request.zero_motion.zupt;
      measurement.zero_rotation_update = request.zero_motion.zaru;
    }
    nav_suite_update(&suite_, &measurement);
  }
}

void Estimator::snapshot(const nvg_ins_v1_EstimatorRequest& request,
                         nvg_ins_v1_EstimatorState& state) const {
  state = nvg_ins_v1_EstimatorState_init_zero;
  state.protocol_version = kProtocolVersion;
  copy_text(state.request_id, request.request_id);
  state.sequence = request.sequence;
  copy_text(state.estimator_session_id, request.estimator_session_id);
  copy_text(state.clock_epoch, request.clock_epoch);
  copy_text(state.origin_epoch, request.origin_epoch);
  state.state_capture_time_us = last_imu_time_us_ > 0
      ? static_cast<std::uint64_t>(last_imu_time_us_)
      : request.mapped_capture_monotonic_ns / 1000;
  state.covariance_capture_time_us = state.state_capture_time_us;
  state.mapped_capture_monotonic_ns = request.mapped_capture_monotonic_ns;
  state.mapped_capture_unix_ns = request.mapped_capture_unix_ns;
  state.timing_uncertainty_ns = request.timing_uncertainty_ns;
  copy_text(state.calibration_hash, request.calibration_hash);

  const auto mode = nav_suite_get_mode(&suite_);
  switch (mode) {
    case NAV_SUITE_MODE_FULL:
      state.estimator_mode = nvg_ins_v1_EstimatorMode_ESTIMATOR_MODE_FULL;
      break;
    case NAV_SUITE_MODE_COASTING:
      state.estimator_mode = nvg_ins_v1_EstimatorMode_ESTIMATOR_MODE_COASTING;
      break;
    case NAV_SUITE_MODE_ATTITUDE_ONLY:
      state.estimator_mode = nvg_ins_v1_EstimatorMode_ESTIMATOR_MODE_ATTITUDE_ONLY;
      break;
    default:
      state.estimator_mode = nvg_ins_v1_EstimatorMode_ESTIMATOR_MODE_NONE;
      break;
  }
  state.aiding_mode = nvg_ins_v1_AidingMode_AIDING_MODE_LOOSE;
  state.ready = mode == NAV_SUITE_MODE_FULL || mode == NAV_SUITE_MODE_COASTING;

  float vector[3]{};
  if (ins_get_position_local(&suite_.ins, vector)) {
    state.has_position_ned_m = true;
    set_vector(state.position_ned_m, vector);
  }
  if (ins_get_velocity_ned(&suite_.ins, vector)) {
    state.has_velocity_ned_mps = true;
    set_vector(state.velocity_ned_mps, vector);
  }
  float quaternion[4]{};
  if (ins_get_quaternion(&suite_.ins, quaternion)) {
    state.has_orientation_body_to_ned = true;
    state.orientation_body_to_ned.w = quaternion[0];
    state.orientation_body_to_ned.x = quaternion[1];
    state.orientation_body_to_ned.y = quaternion[2];
    state.orientation_body_to_ned.z = quaternion[3];
  } else {
    float roll = 0.0F, pitch = 0.0F, yaw = 0.0F;
    if (nav_suite_get_rpy(&suite_, &roll, &pitch, &yaw)) {
      state.has_orientation_body_to_ned = true;
      rpy_to_quaternion(roll, pitch, yaw, state.orientation_body_to_ned);
    }
  }
  if (ins_get_bias_acc(&suite_.ins, vector)) {
    state.has_accelerometer_bias_frd_mps2 = true;
    set_vector(state.accelerometer_bias_frd_mps2, vector);
  }
  if (ins_get_bias_gyr(&suite_.ins, vector)) {
    state.has_gyroscope_bias_frd_rad_s = true;
    set_vector(state.gyroscope_bias_frd_rad_s, vector);
  }
  if (ins_get_bias_mag(&suite_.ins, vector)) {
    state.has_magnetometer_bias_frd_ut = true;
    set_vector(state.magnetometer_bias_frd_ut, vector);
  }

  const auto dimension = static_cast<std::size_t>(suite_.ins.n);
  state.covariance_order_count = static_cast<pb_size_t>(dimension);
  state.covariance_count = static_cast<pb_size_t>(dimension * dimension);
  for (std::size_t index = 0; index < dimension; ++index) {
    copy_text(state.covariance_order[index], kCovarianceOrder[index]);
  }
  for (std::size_t row = 0; row < dimension; ++row) {
    for (std::size_t column = 0; column < dimension; ++column) {
      double covariance = 0.0;
      for (std::size_t k = std::max(row, column); k < dimension; ++k) {
        covariance += static_cast<double>(suite_.ins.U[row + k * dimension]) *
                      static_cast<double>(suite_.ins.d[k]) *
                      static_cast<double>(suite_.ins.U[column + k * dimension]);
      }
      state.covariance[row * dimension + column] = covariance;
    }
  }

  state.actual_output_hz = 0;
  if (request.imu_samples_count >= 2) {
    const auto elapsed = request.imu_samples[request.imu_samples_count - 1].capture_time_us -
                         request.imu_samples[0].capture_time_us;
    if (elapsed > 0) {
      state.actual_output_hz = static_cast<std::uint32_t>(std::min<std::uint64_t>(
          1000, (request.imu_samples_count - 1) * 1'000'000ULL / elapsed));
    }
  }
  const auto* diagnostics = ins_get_diag(&suite_.ins);
  if (diagnostics != nullptr) {
    state.has_diagnostics = true;
    state.diagnostics.last_gnss_position_residual_m = diagnostics->last_gnss_pos_residual_m;
    state.diagnostics.maximum_gnss_position_residual_m = diagnostics->max_gnss_pos_residual_m;
    state.diagnostics.last_gnss_velocity_residual_mps = diagnostics->last_gnss_vel_residual_mps;
    state.diagnostics.rejected_measurements = diagnostics->n_gnss_rejected_noise +
                                              diagnostics->n_fuse_fail +
                                              diagnostics->n_invalid_input;
    state.diagnostics.downweighted_measurements = diagnostics->n_downweighted;
    state.diagnostics.time_resets = diagnostics->n_time_jump_reset +
                                    diagnostics->n_time_restart_reset;
    state.diagnostics.overconfidence_events = diagnostics->n_overconfident;
    state.diagnostics.skipped_samples = diagnostics->n_time_dropped +
                                        diagnostics->n_gnss_rate_limited;
  }
}

bool Estimator::process(const nvg_ins_v1_EstimatorRequest& request,
                        nvg_ins_v1_EstimatorState& state,
                        std::string& error) {
  if (!validate(request, error)) return false;
  const bool epoch_changed = initialized_ &&
      (session_ != request.estimator_session_id || clock_epoch_ != request.clock_epoch ||
       origin_epoch_ != request.origin_epoch);
  if (!initialized_ || epoch_changed) {
    if (!reset(request, error)) return false;
  } else if (request.sequence <= last_sequence_) {
    error = "request sequence is not strictly increasing";
    return false;
  }
  apply(request);
  snapshot(request, state);
  last_sequence_ = request.sequence;
  return true;
}

}  // namespace nvg::ins
