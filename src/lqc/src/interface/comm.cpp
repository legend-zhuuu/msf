#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <lqc/interface/comm.h>

namespace lqc {

Rate::Rate(int freq) : event_time_(clock::now()), cycle_(int(1e9) / freq) {}

void Rate::sleep() {
  event_time_ += cycle_;
  std::this_thread::sleep_until(event_time_);
}

Comm::Comm() {
#if LQC_ROBOT == LQC_ALIENGO
  udp_pub_ = std::make_unique<sdk::UDP>(sdk::LOWLEVEL);
  safe_ = std::make_unique<sdk::Safety>(sdk::LeggedType::Aliengo);
#elif LQC_ROBOT == LQC_GO1
  udp_pub_ = std::make_unique<sdk::UDP>(
      sdk::LOWLEVEL, 8090, "192.168.123.10", 8007);
  safe_ = std::make_unique<sdk::Safety>(sdk::LeggedType::Go1);
#elif LQC_ROBOT == LQC_B1
  udp_pub_ = std::make_unique<sdk::UDP>(
      sdk::LOWLEVEL, 8090, "192.168.123.10", 8007);
  safe_ = std::make_unique<sdk::Safety>(sdk::LeggedType::B1);
#else
#error "Unknown robot type"
#endif  // LQC_ROBOT
  udp_pub_->InitCmdData(low_cmd_msg_);
  low_cmd_msg_.levelFlag = sdk::LOWLEVEL;
  clearCmd();
  udp_pub_->SetSend(low_cmd_msg_);
  udp_pub_->Send();
  ncpus_ = sysconf(_SC_NPROCESSORS_CONF);

  long verbose;
  if (ev::parse("LQC_VERBOSE", verbose) == ev::Success) {
    lg::setLevel(verbose);
  }
}

void Comm::connect(int freq) {
  using namespace std::chrono;
  if (comm_thread_.joinable()) {  // thread has already started
    if (communicating_) communicating_ = false;
    comm_thread_.join();
  }

  active_ = false;
  communicating_ = true;
  comm_freq_ = freq;
  comm_thread_ = std::thread([this]() {
    comm_tid_ = gettid();
    Rate rate(comm_freq_);
    bool printed1 = false, printed2 = false;
    while (communicating_) {
      commEvent();
      if (!printed1 && connected_) {
        log("Robot connected.");
        printed1 = true;
        printed2 = true;
      }
      if (!printed2 && !connected_) {
        lg::crit("Robot not connected.");
        printed2 = true;
      }
      rate.sleep();
    }
  });

  comm_cpuid_ = parseCpuidFromEnvVar("LQC_COMM_CPUID");
  while (comm_tid_ < 0);
  if (comm_cpuid_ >= 0) bindThreadToCpu(comm_tid_, comm_cpuid_);
  int priority = setThreadPriority(comm_thread_.native_handle());
  if (priority > 0) log("Communication thread priority: ", priority, ".");
}

void Comm::disconnect() {
  communicating_ = false;
  connected_ = false;
  if (comm_thread_.joinable()) {
    comm_thread_.join();
    log("Disconnected from Robot.");
  }
}

long Comm::parseCpuidFromEnvVar(const char *name) {
  long cpuid;
  auto status = ev::parse(name, cpuid);
  if (status == ev::Success) {
    if (cpuid < ncpus_) return cpuid;
    lg::crit(name, " >= NCpu(", ncpus_, ").");
    return -1;
  }
  if (status == 2) lg::crit("`", name, "` must be a integer!");
  return -1;
}

int Comm::bindThreadToCpu(__pid_t pid, std::size_t cpuid) const {
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(cpuid, &mask);
  int ret = sched_setaffinity(pid, sizeof(mask), &mask);
  if (ret) {
    lg::warn("Failed to set affinity. Error code: ", ret, ".");
    return -1;
  }
  sched_getaffinity(pid, sizeof(mask), &mask);
  for (int i = 0; i < ncpus_; ++i) {
    if (CPU_ISSET(i, &mask)) return i;
  }
  return -1;
}

int Comm::setThreadPriority(pthread_t thread) {
  sched_param param{sched_get_priority_max(SCHED_FIFO)};
  int ret = pthread_setschedparam(thread, SCHED_FIFO, &param);
  if (ret) {
    lg::warn("Failed to set thread priority. Error code: ", ret, ".");
    return -1;
  }
  int policy;
  pthread_getschedparam(thread, &policy, &param);
  return param.sched_priority;
}

void Comm::commEvent() {
  udp_pub_->Recv();
  comm_mtx_.lock();
  udp_pub_->GetRecv(low_state_msg_);
  connected_ = low_state_msg_.tick != 0;
  if (!frozen_ && !isSafe()) {
    frozen_ = true;
    lg::warn("Locomotion disabled due to unsafe pose!");
  }
  if (frozen_) active_ = false;
  if (!active_) clearCmd();
//  safe_->PowerProtect(low_cmd_msg_, low_state_msg_, 10);
  udp_pub_->SetSend(low_cmd_msg_);
  comm_mtx_.unlock();
  if (active_) udp_pub_->Send();
  commCallback();
}

void Comm::commCallback() {
  comm_mtx_.lock();
#if LQC_ROBOT == LQC_B1
  ArrX joint_pos{12};
  for (int i = 0; i < 12; ++i) {
    joint_pos[i] = low_state_msg_.motorState[i].q;
  }
  joint_pos_buf_.push_back(joint_pos);
#endif  // LQC_ROBOT == LQC_B1
  if (logger_) {
    logger_->pubProprio(low_state_msg_, low_cmd_msg_);
  }
  comm_mtx_.unlock();
  comm_tick_ += 1;
  accel_filter_.update(low_state_msg_);
}

bool Comm::isSafe() const {
  float r = low_state_msg_.imu.rpy[0], p = low_state_msg_.imu.rpy[1];
  return std::abs(r) < M_PI / 3 and std::abs(p) < M_PI / 3;
}

void Comm::logger(std::shared_ptr<Logger> logger) {
  logger_ = std::move(logger);
}

void Comm::clearCmd() {
  for (int i = 0; i < 12; i++) {
    auto &cmd = low_cmd_msg_.motorCmd[i];
    cmd.mode = 0x0A;   // motor switch to servo (PMSM) mode
    cmd.q = sdk::PosStopF;
    cmd.dq = sdk::VelStopF;
    cmd.Kp = cmd.Kd = cmd.tau = 0;
  }
}

}  // namespace lqc
