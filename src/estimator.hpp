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
};

}  // namespace nvg::ins
