#include <lqc/interface/agent.h>

namespace lqc {

Agent::Agent() {
  action_.setZero();
  stand_ << 0., 0.6435, -1.287, 0., 0.6435, -1.287,
      0., 0.6435, -1.287, 0., 0.6435, -1.287;
  lie_ << 0., 1.343, -2.696, 0., 1.343, -2.696,
      0., 1.343, -2.696, 0., 1.343, -2.696;
}

void Agent::policy(std::shared_ptr<Policy> policy) {
  policy_ = std::move(policy);
  if (logger_) policy_->logger(logger_);
}

void Agent::start() {
  connect(1000);
  startAgent(200);
}

void Agent::startAgent(int freq) {
  if (agent_thread_.joinable()) {
    if (telecontrol_) telecontrol_ = false;
    agent_thread_.join();
  }
  policy_freq_ = freq;
  telecontrol_ = true;
  agent_thread_ = std::thread([this]() {
    using namespace std::chrono;
    agent_tid_ = gettid();
    // Waiting for connecting to robot
    while (!connected_) {
      std::this_thread::sleep_for(1ms);
      if (!telecontrol_) return;
    }

    // Waiting for connecting to gamepad
    bool printed = false;
    while (!gamepad_.connected()) {
      if (!printed) {
        lg::warn("Plug in gamepad to start.");
        printed = true;
      }
      std::this_thread::sleep_for(50ms);
      if (!telecontrol_) return;
    }

    log("Agent started.");
    int tick = comm_tick_;
    while (telecontrol_) {
      while (comm_tick_ < tick) std::this_thread::sleep_for(10us);
      if (!connected_) {
        lg::crit("Agent failure due to interrupted communication.");
        break;
      }
      policyEvent();
      tick += int(comm_freq_ / policy_freq_);
    }
  });

  agent_cpuid_ = parseCpuidFromEnvVar("LQC_AGENT_CPUID");
  while (agent_tid_ < 0);
  if (agent_cpuid_ >= 0) bindThreadToCpu(agent_tid_, agent_cpuid_);
  int priority = setThreadPriority(agent_thread_.native_handle());
  if (priority > 0) log("Agent thread priority: ", priority, ".");
}

void Agent::stopAgent() {
  if (agent_thread_.joinable()) {
    telecontrol_ = false;
    agent_thread_.join();
    log("Agent stopped.");
  }
}

void Agent::policyEvent() {
  updateGamepadCmd();
  updateLocoState();
}

void Agent::updateGamepadCmd() {
  gamepad_.getState(gp_cmd_);
  if (gp_cmd_.LAS.pressed && gp_cmd_.RAS.pressed) {
    if (!frozen_) {
      setLocoState(LocoState::Lie);
      log("Agent disabled.");
      frozen_ = true;
    }
    return;
  }

  if (gp_cmd_.lt > 0.99) {
    if (gp_cmd_.A) {
      if (loco_state_ == LocoState::Lie) {
        setLocoState(LocoState::L2S);
      } else if (loco_state_ == LocoState::L2S) {
        setLocoState(LocoState::Lie);
      } else if (loco_state_ == LocoState::Stand) {
        setLocoState(LocoState::S2L);
      }
    } else if (gp_cmd_.B) {
      if (loco_state_ == LocoState::Stand) {
        setLocoState(LocoState::S2M);
      } else if (loco_state_ == LocoState::Move) {
        setLocoState(LocoState::M2S);
      }
    } else if (gp_cmd_.X) {
      if (frozen_) {
        setLocoState(LocoState::Lie);
        log("Agent recovered.");
        frozen_ = false;
      }
    }
  }
}

void Agent::updateLocoState() {
  if (frozen_) return;
  LocoState next_state = loco_state_;
  switch (loco_state_) {
    case LocoState::Lie: {
      active_ = false;
      break;
    }
    case LocoState::Stand: {
      action_ = stand_;
      break;
    }
    case LocoState::Move: {
      if (!policyAct()) {
        lg::crit("Policy failure. Switch to Lying.");
        next_state = LocoState::Lie;
      }
      break;
    }
    case LocoState::L2S: {  // from lying to standing
      next_state = eventL2S();
      break;
    }
    case LocoState::S2L: {  // from standing to lying
      next_state = eventS2L();
      break;
    }
    case LocoState::S2M: {  // from standing to moving
      next_state = eventS2M();
      break;
    }
    case LocoState::M2S: {  // from moving to standing
      next_state = eventM2S();
      break;
    }
    default: {
      LQC_ERROR("Unintended condition!")
    }
  }

  if (next_state != loco_state_) {
    if (loco_state_ == LocoState::Move) {
      log("Average policy time: ", policy_timer_.mean<timer::us>(), "us.");
    }
    setLocoState(next_state);
  } else {
    ++state_tick_;
  }
}

void Agent::setJointCmd(float kp, float kd) {
  LockGuard lock(comm_mtx_);
  for (int i = 0; i < 12; ++i) {
    auto &cmd = low_cmd_msg_.motorCmd[i];
    cmd.q = action_[i];
    cmd.Kp = kp;
    cmd.Kd = kd;
    cmd.dq = cmd.tau = 0.;
  }
}

void Agent::setJointCmd(cArrX kp, cArrX kd) {
  LockGuard lock(comm_mtx_);
  for (int i = 0; i < 12; ++i) {
    auto &cmd = low_cmd_msg_.motorCmd[i];
    cmd.q = action_[i];
    cmd.Kp = kp[i];
    cmd.Kd = kd[i];
    cmd.dq = cmd.tau = 0.;
  }
}

bool Agent::policyInit() {
  if (!policy_) {
    lg::crit("Policy is not set!");
    return false;
  }
  policy_freq_ = policy_->getFreq();
  accel_filter_.reset();
  policy_timer_.clear();
  return policy_->init();
}

bool Agent::policyAct() {
  if (!policy_) return false;
  timer::TimerGuard _(policy_timer_);
  std::map<RoboState, ArrX> state_map;
  comm_mtx_.lock();
  state_map[RoboState::Gyro] = Eigen::Map<Arr3>(low_state_msg_.imu.gyroscope.data());
  state_map[RoboState::Accel] = accel_filter_.getFilteredAccel();
  state_map[RoboState::Rpy] = Eigen::Map<Arr3>(low_state_msg_.imu.rpy.data());
#if LQC_ROBOT == LQC_GO1  // TODO: add to config file
  state_map[RoboState::Rpy] += Arr3(1.8 / 180 * M_PI, -1.5 / 180 * M_PI, 0.);
#endif  // LQC_ROBOT == LQC_GO1
  state_map[RoboState::Orn] = Eigen::Map<Arr4>(low_state_msg_.imu.quaternion.data());

  Arr<12> joint_pos, joint_vel;
  for (int i = 0; i < 12; ++i) {
    joint_pos[i] = low_state_msg_.motorState[i].q;
    joint_vel[i] = low_state_msg_.motorState[i].dq;
  }
#if LQC_ROBOT == LQC_B1
  // manually calculate joint velocity
  // because of the high latency of robot feedback
  if (joint_pos_buf_.is_full()) {
    joint_vel = (joint_pos_buf_.back() - joint_pos_buf_.front())
        * float(comm_freq_) / float(joint_pos_buf_.size() - 1);
  }
#endif  // LQC_ROBOT == LQC_B1
  state_map[RoboState::JnPos] = joint_pos;
  state_map[RoboState::JnVel] = joint_vel;
  comm_mtx_.unlock();

  ArrX kp{12}, kd{12};
  if (!policy_->act(state_map, gp_cmd_, action_, kp, kd)) {
    action_ = stand_;
    return false;
  }
  setJointCmd(kp, kd);
  return true;
}

LocoState Agent::eventL2S() {
  Arr<12> joint_pos;
  comm_mtx_.lock();
  for (int i = 0; i < 12; ++i) {
    joint_pos[i] = low_state_msg_.motorState[i].q;
  }
  comm_mtx_.unlock();

  if (state_tick_ == 0) {
    log("Standing up...");
    action_ = joint_pos;
    active_ = true;
  }
  int num_steps_ph1 = 0.8 * policy_freq_,
      num_steps_ph2 = 1.2 * policy_freq_;
  if (state_tick_ < num_steps_ph1) {
    interpTo(lie_, state_tick_, num_steps_ph1);
    if (((action_ - joint_pos).abs() > 0.4).any()) {
      lg::warn("Standing failed due to potentially stuck legs!");
      return LocoState::Lie;
    }
    setJointCmd(cfg_.kp_l, cfg_.kd_l);
    return LocoState::L2S;
  } else if (state_tick_ - num_steps_ph1 < num_steps_ph2) {
    interpTo(stand_, state_tick_ - num_steps_ph1, num_steps_ph2);
    setJointCmd(cfg_.kp_h, cfg_.kd_h);
    return LocoState::L2S;
  }
  log("Standing.");
  return LocoState::Stand;
}

LocoState Agent::eventS2L() {
  if (state_tick_ == 0) {
    log("Lying down...");
  }
  int num_steps = 1. * policy_freq_;
  if (state_tick_ < num_steps) {
    interpTo(lie_, state_tick_, num_steps);
    setJointCmd(cfg_.kp_h, cfg_.kd_h);
    return LocoState::S2L;
  }
  log("Lying.");
  return LocoState::Lie;
}

LocoState Agent::eventS2M() {
  if (!policyInit()) {
    lg::crit("Policy initialization failed.");
    return LocoState::Stand;
  }
  log("Policy started.");
  return LocoState::Move;
}

LocoState Agent::eventM2S() {
  if (state_tick_ == 0) {
    log("Policy quited.");
    LockGuard lock(comm_mtx_);
    for (int i = 0; i < 12; ++i) {
      action_[i] = low_state_msg_.motorState[i].q;
    }
  }
  int num_steps = 0.3 * policy_freq_;
  if (state_tick_ < num_steps) {
    interpTo(stand_, state_tick_, num_steps);
    setJointCmd(cfg_.kp_h, cfg_.kd_h);
    return LocoState::M2S;
  }
  log("Standing.");
  return LocoState::Stand;
}

void Agent::setLocoState(LocoState loco_state) {
  loco_state_ = loco_state;
  state_tick_ = 0;
}

void Agent::interpTo(cArrX goal, int curr_steps, int total_steps) {
  action_ += (goal - action_) / (total_steps - curr_steps);
}

}  // namespace lqc
