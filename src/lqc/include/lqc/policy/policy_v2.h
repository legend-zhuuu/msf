#ifndef LQC_POLICY_V2_H_
#define LQC_POLICY_V2_H_

#include <fstream>
#include <utility>
#include <lqc/interface/agent.h>
#include <lqc/runtime/runtime.h>

namespace lqc {

namespace ob {
enum Item {
  CmdVel, CmdStand, Accel, RollPitch, Gravity,
  JnPos, JnVel, LinVel, AngVel, LastAction,
  HeightScan,
};
extern const std::map<std::string, Item> kItemMap;
extern const std::map<Item, std::string> kNameMap;
extern std::map<Item, int> kSizeMap;

void parseFromYaml(
    const YAML::Node &node,
    std::vector<Item> &result);
void construct(
    const std::map<Item, ArrX> &ob_map,
    const std::vector<Item> &ob_list,
    rArrX result);
void split(
    const Eigen::Ref<const ArrX> &ob_vec,
    const std::vector<Item> &ob_list,
    std::map<Item, ArrX> &result);
}

using Observation = ob::Item;

class Sensor {
 public:
  virtual void sense(std::set<Observation> &result) = 0;
  virtual bool init(std::set<Observation> &requirements) = 0;
  virtual bool update(const std::map<RoboState, ArrX> &state_map,
                      const Gamepad::State &gp_state,
                      std::map<Observation, ArrX> &ob_map) = 0;

  void logger(std::shared_ptr<Logger> logger) {
    logger_ = std::move(logger);
  }

 protected:
  std::shared_ptr<Logger> logger_{nullptr};
};

class CmdVelSensor : public Sensor {
 public:
  CmdVelSensor() = default;
  explicit CmdVelSensor(const YAML::Node &cfg);
  void sense(std::set<Observation> &result) override;
  bool init(std::set<Observation> &requirements) override;
  bool update(const std::map<RoboState, ArrX> &state_map,
              const Gamepad::State &gp_state,
              std::map<Observation, ArrX> &ob_map) override;

 protected:
  Arr3 scale_{1., 0.5, 1.}, ratio_{2., 2., 2.};
  bool smooth_{false};
  Arr3 smooth_rate_{0.06, 0.02, 0.1};
  Arr3 cmd_vel_;
  Arr<1> cmd_stand_;
};

class Estimator : public Sensor {
 public:
  explicit Estimator(const std::string &path);
  void sense(std::set<Observation> &result) override;
  bool init(std::set<Observation> &requirements) override;
  bool update(const std::map<RoboState, ArrX> &state_map,
              const Gamepad::State &gp_state,
              std::map<Observation, ArrX> &ob_map) override;

 private:
  runtime::RNN estimator_;
  std::vector<Observation> ob_list_, estimation_list_;
  YAML::Node estimator_cfg_;
  ArrX estimator_ob_;
  int ob_dim_, estimation_dim_;
};

class PolicyV2 : public Policy {
 public:
  explicit PolicyV2(const std::string &path);

  bool requires(Observation ob) const;
  void sensor(std::shared_ptr<Sensor> sensor);
  template<typename T, typename ...Args>
  void sensor(const Args &...args) { sensor(std::make_shared<T>(args...)); }
  void logger(std::shared_ptr<Logger> logger) override;

  int getFreq() const override { return freq_; }
  bool init() override;
  bool act(const std::map<State, ArrX> &state_map,
           const Gamepad::State &gp_state,
           rArrX action, rArrX kp, rArrX kd) override;

 private:
  void addSensors(const std::string &path);
  void initSensors(std::set<Observation> &result);
  void updateSensors(
      const std::map<State, ArrX> &state_map,
      const Gamepad::State &gp_state,
      std::map<Observation, ArrX> &result);
  std::vector<std::shared_ptr<Sensor>> sensors_;
  std::set<Observation> senses_;

  runtime::RNN policy_;
  std::vector<Observation> ob_list_;
  YAML::Node policy_cfg_;
  int ob_dim_, action_dim_;
  ArrX ob_, last_action_;
  ArrX action_mean_, action_std_;

  ArrX kp_, kd_;
  int freq_;
  std::string name_;
  size_t num_steps_{0}, warmup_steps_{0};
  bool denormalized_{true};
};

}  // namespace lqc

#endif  // LQC_POLICY_V2_H_
