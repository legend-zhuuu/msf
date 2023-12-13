#ifndef LQC_AGENT_H_
#define LQC_AGENT_H_

#include <map>
#include <set>
#include <utility>

#include <lqc/interface/comm.h>
#include <lqc/interface/gamepad.h>

namespace lqc {

enum class LocoState {
  Lie, L2S, S2L, Stand, S2M, M2S, Move
};
enum class RoboState {
  Gyro, Accel, Rpy, Orn, JnPos, JnVel
};

class Policy {
 public:
  using State = RoboState;
  virtual int getFreq() const = 0;
  virtual bool init() = 0;
  virtual bool act(const std::map<State, ArrX> &state_map,
                   const Gamepad::State &gp_state,
                   rArrX action, rArrX kp, rArrX kd) = 0;
  virtual void logger(std::shared_ptr<Logger> logger) {
    logger_ = std::move(logger);
  }

 protected:
  std::shared_ptr<Logger> logger_{nullptr};
};

class Agent : public Comm {
 public:
  explicit Agent();
  ~Agent() { stopAgent(); }

  void policy(std::shared_ptr<Policy> policy);
  template<typename T, typename ...Args>
  void policy(const Args &...args) {
    policy(std::shared_ptr<Policy>(new T{args...}));
  }

  void start();
  void startAgent(int freq);
  void stopAgent();

 protected:
  bool policyInit();
  bool policyAct();
  void interpTo(cArrX goal, int curr_steps, int total_steps);

  Gamepad::State gp_cmd_{};
  std::thread agent_thread_;
  ArrX action_{12}, stand_{12}, lie_{12};
  std::atomic<bool> telecontrol_{false};
  LocoState loco_state_{LocoState::Lie};

 private:
  void policyEvent();
  void updateGamepadCmd();
  void updateLocoState();
  void setLocoState(LocoState loco_state);

  LocoState eventL2S();
  LocoState eventS2L();
  LocoState eventS2M();
  LocoState eventM2S();
  void setJointCmd(float kp, float kd);
  void setJointCmd(cArrX kp, cArrX kd);

  std::atomic<int> policy_freq_{200};
  std::atomic<pid_t> agent_tid_{-1};
  long agent_cpuid_{-1};
  Gamepad gamepad_;
  int state_tick_ = 0;
  std::shared_ptr<Policy> policy_{nullptr};
  timer::Timer policy_timer_;

  struct Config {
#if LQC_ROBOT == LQC_ALIENGO
    float kp_l{50.}, kd_l{1.5};
    float kp_h{150.}, kd_h{4.};
#elif LQC_ROBOT == LQC_GO1
    float kp_l{50.}, kd_l{1.5};
    float kp_h{100.}, kd_h{2.};
#elif LQC_ROBOT == LQC_B1
    float kp_l{150.}, kd_l{3.};
    float kp_h{400.}, kd_h{4.};
#endif  // LQC_ROBOT
  } cfg_;
};

}  // namespace lqc

#endif  // LQC_AGENT_H_
