#ifndef LQC_LEGACY_POLICY_H_
#define LQC_LEGACY_POLICY_H_

#include <lqc/interface/agent.h>
#include <lqc/runtime/runtime.h>

namespace lqc {

class LegacyPolicy : public Policy {
 public:
  explicit LegacyPolicy(const std::string &path);

  int getFreq() const override { return 200; }
  bool init() override;
  bool act(const std::map<State, ArrX> &state_map,
           const Gamepad::State &gp_state,
           rArrX action, rArrX kp, rArrX kd) override;

 private:
  runtime::RNN policy_, estimator_;
  ArrX proprio_{29}, obs_{36};
  ArrX action_mean_{12};
  float action_std_{0.8};
  Arr3 cmd_vel_scale_{2., 1., 2.};
};

}  // namespace lqc

#endif  // LQC_LEGACY_POLICY_H_
