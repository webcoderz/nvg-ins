#pragma once

#include <cstdint>
#include <string>

extern "C" {
#include "ins.pb.h"
#include "nav_suite.h"
}

namespace nvg::ins {

class Estimator final {
 public:
  Estimator();

  /** How long the filter may coast on IMU alone before `ins_is_ready` degrades.
   *
   * INSLIB's default is a few seconds, which suits a vehicle that expects GNSS back shortly.
   * A wearer walking through a building, a tunnel or smoke has no such expectation: the
   * estimator coasting is the whole capability, so the window is stated by the operator rather
   * than inherited. Zero keeps INSLIB's default; a negative value means unlimited, which
   * INSLIB supports explicitly and which must be chosen deliberately, since a solution that
   * never degrades is a solution that never admits it is lost.
   */
  void set_max_deadreckoning_sec(float seconds);
  bool process(const nvg_ins_v1_EstimatorRequest& request,
               nvg_ins_v1_EstimatorState& state,
               std::string& error);

 private:
  bool reset(const nvg_ins_v1_EstimatorRequest& request, std::string& error);
  bool validate(const nvg_ins_v1_EstimatorRequest& request, std::string& error) const;
  void apply(const nvg_ins_v1_EstimatorRequest& request);
  void snapshot(const nvg_ins_v1_EstimatorRequest& request,
                nvg_ins_v1_EstimatorState& state) const;

  nav_suite_t suite_{};
  std::string session_;
  std::string clock_epoch_;
  std::string origin_epoch_;
  std::uint64_t last_sequence_{0};
  std::int64_t last_imu_time_us_{0};
  bool initialized_{false};
  float max_deadreckoning_sec_{0.0F};
};

}  // namespace nvg::ins
