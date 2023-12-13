#include <lqc/policy/legacy_policy.h>

namespace lqc {

LegacyPolicy::LegacyPolicy(const std::string &path) {
  log("──────── [\033[1;32mLegacyPolicy\033[0m] ────────");
  policy_.loadModel(path + "/lstm.onnx");
  assert(policy_.getInputSize("x") == 36);

  log("───────── [\033[1;32mEstimator\033[0m] ──────────");
  estimator_.loadModel(path + "/estimator.onnx");
  assert(estimator_.getInputSize("x") == 29);
  log("────────────────────────────────");
  action_mean_ << 0., 0.6435, -1.287, 0., 0.6435, -1.287,
      0., 0.6435, -1.287, 0., 0.6435, -1.287;
}

bool LegacyPolicy::init() {
  policy_.clearStates();
  estimator_.clearStates();
  return true;
}

bool LegacyPolicy::act(
    const std::map<State, ArrX> &state_map,
    const Gamepad::State &gp_state,
    rArrX action, rArrX kp, rArrX kd) {
  proprio_.setZero();
  proprio_.head<2>() << state_map.at(State::Rpy).head<2>();
  proprio_.segment<12>(2) << state_map.at(State::JnPos);
  proprio_.segment<12>(14) << state_map.at(State::JnVel);
  proprio_.segment<3>(26) << state_map.at(State::Gyro);
  estimator_.setInput("x", proprio_.data());
  estimator_.run();
  Arr3 lin_vel = Eigen::Map<const Arr3>(estimator_.getOutput("y"));

  obs_.setZero();
  Arr3 cmd_vel(-gp_state.las_y, -gp_state.las_x, -gp_state.ras_x);
  cmd_vel.head<2>() *= sqrt(1 - cmd_vel[2] * cmd_vel[2]);
  float bonus = 1.5f + gp_state.rt * 0.5f;
  cmd_vel *= bonus * cmd_vel_scale_;

  obs_.head<3>() << cmd_vel;
  obs_.segment<2>(3) << state_map.at(State::Rpy).head<2>();
  obs_.segment<12>(5) << state_map.at(State::JnPos);
  obs_.segment<12>(17) << state_map.at(State::JnVel);
  obs_.segment<3>(29) << state_map.at(State::Gyro);
  obs_.segment<3>(32) << lin_vel;
  bool stand = (cmd_vel.abs() < 0.1).all();
  if (stand) obs_.head<3>().setZero();
  obs_[35] = stand;
  policy_.setInput("x", obs_.data());
  policy_.run();
  auto y = Eigen::Map<const ArrX>(policy_.getOutput("y"), 12);
  action = y * action_std_ + action_mean_;
  kp.setConstant(30.f);
  kd.setConstant(0.5f);
  return true;
}

}  // namespace lqc
