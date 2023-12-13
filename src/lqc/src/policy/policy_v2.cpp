#include <lqc/policy/policy_v2.h>
#include <lqc/utils/yaml.h>

namespace lqc {
namespace ob {
const std::map<std::string, Item> kItemMap{
    {"cmd_vel", CmdVel},
    {"cmd_standing", CmdStand},
    {"lin_acc", Accel},
    {"roll_pitch", RollPitch},
    {"gravity", Gravity},
    {"joint_pos", JnPos},
    {"joint_vel", JnVel},
    {"lin_vel", LinVel},
    {"ang_vel", AngVel},
    {"last_action", LastAction},
    {"height_scan", HeightScan},
};

const std::map<Item, std::string> kNameMap{
    {CmdVel, "cmd_vel"},
    {CmdStand, "cmd_standing"},
    {Accel, "lin_acc"},
    {RollPitch, "roll_pitch"},
    {Gravity, "gravity"},
    {JnPos, "joint_pos"},
    {JnVel, "joint_vel"},
    {LinVel, "lin_vel"},
    {AngVel, "ang_vel"},
    {LastAction, "last_action"},
    {HeightScan, "height_scan"},
};

std::map<Item, int> kSizeMap{
    {CmdVel, 3},
    {CmdStand, 1},
    {LinVel, 3},
    {RollPitch, 2},
    {Gravity, 3},
    {JnPos, 12},
    {JnVel, 12},
    {LinVel, 3},
    {AngVel, 3},
    {LastAction, 12},
    {HeightScan, -1},
};

void parseFromYaml(
    const YAML::Node &node,
    std::vector<Item> &result) {
  assert(node.IsSequence());
  for (auto item : node) {
    auto name = item.as<std::string>();
    auto it = ob::kItemMap.find(name);
    if (it == ob::kItemMap.end()) {
      LQC_ERROR("Unrecognized observation `" + name + "`.")
    }
    result.push_back(it->second);
  }
}

inline void stackVec(cArrX vec, long &dim, rArrX result) {
  result.segment(dim, vec.size()) << vec;
  dim += vec.size();
}

void construct(
    const std::map<Item, ArrX> &ob_map,
    const std::vector<Item> &ob_list,
    rArrX result) {
  long dim = 0;
  for (auto item : ob_list) {
    stackVec(ob_map.at(item), dim, result);
  }
  assert(dim == result.size());
}

void split(
    const Eigen::Ref<const ArrX> &ob_vec,
    const std::vector<Item> &ob_list,
    std::map<Item, ArrX> &result) {
  int dim = 0;
  for (auto item : ob_list) {
    int size = kSizeMap.at(item);
    if (size < 0) {
      LQC_ERROR("Size of `" + kNameMap.at(item) + "` is undefined.")
    }
    result[item] = ob_vec.segment(dim, size);
    dim += size;
  }
}

}  // namespace ob


CmdVelSensor::CmdVelSensor(const YAML::Node &cfg) {
  yaml::setIfValid(cfg, "scale", scale_);
  yaml::setIfValid(cfg, "ratio", ratio_);
  yaml::setIfValid(cfg, "smooth", smooth_);
  yaml::setIfValid(cfg, "smooth_rate", smooth_rate_);
}

void CmdVelSensor::sense(std::set<Observation> &result) {
  result.insert(ob::CmdVel);
  result.insert(ob::CmdStand);
}

bool CmdVelSensor::init(std::set<Observation> &requirements) {
  cmd_vel_.setZero();
  return true;
}

bool CmdVelSensor::update(
    const std::map<RoboState, ArrX> &state_map,
    const Gamepad::State &gp_state,
    std::map<Observation, ArrX> &ob_map) {
  float lin_x = -gp_state.las_y, lin_y = -gp_state.las_x, rot_z = -gp_state.ras_x;
  Arr3 cmd_vel{lin_x, lin_y, rot_z};
  cmd_vel *= 1 + scale_ * (ratio_ - 1) * (gp_state.rt / 2 + 0.5);

  if (smooth_) {
    cmd_vel_ += (cmd_vel - cmd_vel_)
        .cwiseMin(smooth_rate_).cwiseMax(-smooth_rate_);
  } else {
    cmd_vel_ = cmd_vel;
  }

  if (cmd_vel_.abs().maxCoeff() < 0.1) {
    ob_map[ob::CmdVel] = Arr3::Zero();
    cmd_stand_.setOnes();
  } else {
    ob_map[ob::CmdVel] = cmd_vel_;
    cmd_stand_.setZero();
  }
  ob_map[ob::CmdStand] = cmd_stand_;
  return true;
}

Estimator::Estimator(const std::string &path) {
  using namespace yaml;
  log("─────── [\033[1;32mEstimator\033[0m] ────────");
  estimator_cfg_ = YAML::LoadFile(path + "/estimator.yml");
  assert(readAs<std::string>(estimator_cfg_, "model_type") == "lstm");
  setTo(estimator_cfg_, "input_dim", ob_dim_);
  setTo(estimator_cfg_, "output_dim", estimation_dim_);

  estimator_.loadModel(path + "/estimator.onnx");
  assert(estimator_.getInputSize("x") == ob_dim_);
  assert(estimator_.getOutputSize("y") == estimation_dim_);

  ob::parseFromYaml(estimator_cfg_["observations"], ob_list_);
  ob::parseFromYaml(estimator_cfg_["estimations"], estimation_list_);
  estimator_ob_.setZero(ob_dim_);

  if (lg::inMode(lg::DBUG)) {
    log("Observations:");
    for (auto item : ob_list_) {
      log("- ", ob::kNameMap.at(item));
    }
  }
}

void Estimator::sense(std::set<Observation> &result) {
  for (auto item : estimation_list_) result.insert(item);
}

bool Estimator::init(std::set<Observation> &requirements) {
  for (auto item : ob_list_) requirements.insert(item);
  estimator_.clearStates();
  return true;
}

bool Estimator::update(
    const std::map<RoboState, ArrX> &state_map,
    const Gamepad::State &gp_state,
    std::map<Observation, ArrX> &ob_map) {
  ob::construct(ob_map, ob_list_, estimator_ob_);
  estimator_.setInput("x", estimator_ob_.data());
  estimator_.run();
  const auto *estimation = estimator_.getOutput("y");
  ob::split(Eigen::Map<const ArrX>(estimation, estimation_dim_),
            estimation_list_, ob_map);

  if (logger_ && lg::inMode(lg::DBUG)) {
    for (auto item : estimation_list_) {
      logger_->pubArray("estimator/" + ob::kNameMap.at(item), ob_map.at(item));
    }
    if (std::find(estimation_list_.begin(),
                  estimation_list_.end(),
                  ob::LinVel) != estimation_list_.end()) {
      logger_->pubTwist(ob_map.at(ob::LinVel), state_map.at(RoboState::Gyro));
      logger_->pubOdom(
          Vec3::Zero(), state_map.at(RoboState::Orn),
          ob_map.at(ob::LinVel), state_map.at(RoboState::Gyro)
      );
    }
  }
  return true;
}

PolicyV2::PolicyV2(const std::string &path) {
  using namespace yaml;
  lg::enableTimeStampPrefix(false);
  log("──────── [\033[1;32mPolicyV2\033[0m] ────────");
  policy_cfg_ = YAML::LoadFile(path + "/policy.yml");
  assert(readAs<std::string>(policy_cfg_, "version") == "v2");
  assert(readAs<std::string>(policy_cfg_, "model_type") == "lstm");

  setTo(policy_cfg_, "observation_dim", ob_dim_);
  setTo(policy_cfg_, "action_dim", action_dim_);
  assert(action_dim_ == 12);
  kp_.setZero(action_dim_);
  kd_.setZero(action_dim_);
  action_mean_.setZero(action_dim_);
  action_std_.setOnes(action_dim_);

  setTo(policy_cfg_, "Kp", kp_);
  setTo(policy_cfg_, "Kd", kd_);
  setTo(policy_cfg_, "freq", freq_);
  setTo(policy_cfg_, "action_mean", action_mean_);
  name_ = readDefault<std::string>(policy_cfg_["run"], "name", "anonymous");
  setIfValid(policy_cfg_, "denormalized", denormalized_);
  if (!denormalized_) {
    setTo(policy_cfg_, "action_std", action_std_);
  }
  setIfValid(policy_cfg_, "warmup_steps", warmup_steps_);
  log("PolicyV2 (", name_, ") runs at ", freq_, "Hz.");

  policy_.loadModel(path + "/policy.onnx");
  assert(policy_.getInputSize("x") == ob_dim_);
  assert(policy_.getOutputSize("y") == action_dim_);
  last_action_.setZero(action_dim_);

  ob::parseFromYaml(policy_cfg_["observations"], ob_list_);
  ob_.setZero(ob_dim_);
  senses_.insert(ob::Accel);
  senses_.insert(ob::RollPitch);
  senses_.insert(ob::Gravity);
  senses_.insert(ob::JnPos);
  senses_.insert(ob::JnVel);
  senses_.insert(ob::AngVel);
  senses_.insert(ob::LastAction);

  if (lg::inMode(lg::DBUG)) {
    log("Observations:");
    for (auto item : ob_list_) {
      log("- ", ob::kNameMap.at(item));
    }
  }

  addSensors(path);
  log("────────────────────────────");
  lg::enableTimeStampPrefix(true);
}

bool PolicyV2::requires(Observation ob) const {
  return std::find(ob_list_.begin(), ob_list_.end(), ob) != ob_list_.end();
}

void PolicyV2::logger(std::shared_ptr<Logger> logger) {
  Policy::logger(logger);
  for (auto &sensor : sensors_) sensor->logger(logger);
}

void PolicyV2::sensor(std::shared_ptr<Sensor> sensor) {
  sensor->sense(senses_);
  if (logger_) sensor->logger(logger_);
  sensors_.push_back(std::move(sensor));
}

bool PolicyV2::init() {
  std::set<Observation> ob_set;
  for (auto item : ob_list_) ob_set.insert(item);
  initSensors(ob_set);
  for (auto item : ob_set) {
    if (senses_.find(item) != senses_.end()) break;
    lg::crit("No sensor for observation `", ob::kNameMap.at(item), "`.");
    return false;
  }
  policy_.clearStates();
  num_steps_ = 0;
  last_action_ = action_mean_;
  return true;
}

bool PolicyV2::act(
    const std::map<State, ArrX> &state_map,
    const Gamepad::State &gp_state,
    rArrX action, rArrX kp, rArrX kd) {
  std::map<Observation, ArrX> ob_map;
  updateSensors(state_map, gp_state, ob_map);
  ob::construct(ob_map, ob_list_, ob_);
  policy_.setInput("x", ob_.data());
  policy_.run();
  action = Eigen::Map<const ArrX>(policy_.getOutput("y"), 12);
  last_action_ = action;
  if (num_steps_ < warmup_steps_) {
    action = action_mean_;
  } else if (!denormalized_) {
    action = action * action_std_ + action_mean_;
  }
  kp = kp_;
  kd = kd_;
  ++num_steps_;

  if (logger_ && lg::inMode(lg::DBUG)) {
    logger_->pubArray("policy/observation", ob_);
    logger_->pubArray("policy/action", action);
    if (policy_.hasOutput("aux")) {
      auto aux = Eigen::Map<const ArrX>(
          policy_.getOutput("aux"),
          policy_.getOutputSize("aux")
      );
      logger_->pubArray("policy/auxiliary_output", aux);
    }
  }
  return true;
}

void PolicyV2::addSensors(const std::string &path) {
  if (std::ifstream(path + "/estimator.yml").good()) {
    sensor<Estimator>(path);
  }
}

void PolicyV2::initSensors(std::set<Observation> &result) {
  for (const auto &sensor : sensors_) sensor->init(result);
}

void PolicyV2::updateSensors(
    const std::map<State, ArrX> &state_map,
    const Gamepad::State &gp_state,
    std::map<Observation, ArrX> &result) {
  result[ob::Accel] = state_map.at(RoboState::Accel);
  result[ob::RollPitch] = state_map.at(RoboState::Rpy).head<2>();
  result[ob::Gravity] = Quaternion(state_map.at(RoboState::Orn)).inv().rotate(Vec3{0, 0, -1.});
  result[ob::JnPos] = state_map.at(RoboState::JnPos);
  result[ob::JnVel] = state_map.at(RoboState::JnVel);
  result[ob::AngVel] = state_map.at(RoboState::Gyro);
  result[ob::LastAction] = last_action_;
  for (const auto &sensor : sensors_) {
    sensor->update(state_map, gp_state, result);
  }
}

}  // namespace lqc
