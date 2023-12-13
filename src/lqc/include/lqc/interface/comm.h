#ifndef LQC_COMM_H_
#define LQC_COMM_H_

#include <unistd.h>
#include <syscall.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>

#include <lqc/utils/utils.h>

#ifdef LQC_FIX_GETTID
#define gettid() syscall(SYS_gettid)
#endif  // LQC_FIX_GETTID

namespace lqc {

class Logger {
 public:
  virtual void pubProprio(const sdk::LowState &, const sdk::LowCmd &) = 0;
  virtual void pubArray(const std::string &name, cArrX) = 0;
  virtual void pubTwist(Vec3 lin_vel, Vec3 ang_vel) = 0;
  virtual void pubOdom(Vec3 pos, Vec4 orn, Vec3 lin_vel, Vec3 ang_vel) = 0;
};

class Rate {
 public:
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using nanoseconds = std::chrono::nanoseconds;
  explicit Rate(int freq);
  void sleep();

 private:
  nanoseconds cycle_;
  time_point event_time_;
};

class Comm {
 public:
  explicit Comm();
  ~Comm() { disconnect(); }

  void logger(std::shared_ptr<Logger> logger);
  template<typename T, typename ...Args>
  void logger(const Args &...args) {
    logger_ = std::make_shared<T>(args...);
  }

  void connect(int freq);
  void disconnect();
  bool isSafe() const;

 protected:
  virtual void commCallback();
  void clearCmd();

  long parseCpuidFromEnvVar(const char *name);
  int bindThreadToCpu(__pid_t pid, std::size_t cpuid) const;
  static int setThreadPriority(pthread_t thread);

  std::shared_ptr<Logger> logger_{nullptr};
  std::atomic<bool> communicating_{false}, connected_{false};
  std::atomic<bool> active_{false}, frozen_{false};
  sdk::LowState low_state_msg_{};
  sdk::LowCmd low_cmd_msg_{};
  std::mutex comm_mtx_;  // for low_cmd_msg_ & low_state_msg_
  long comm_cpuid_{-1};
  std::size_t ncpus_{0};
  std::atomic<int> comm_tick_{0};
  std::atomic<int> comm_freq_{1000};
  SimpleAccelFilter accel_filter_;
#if LQC_ROBOT == LQC_B1
  StaticQueue<ArrX> joint_pos_buf_{6};
#endif  // LQC_ROBOT == LQC_B1

 private:
  void commEvent();
  std::thread comm_thread_;
  std::unique_ptr<sdk::UDP> udp_pub_;
  std::unique_ptr<sdk::Safety> safe_;
  std::atomic<pid_t> comm_tid_{-1};
};

}  // namespace lqc

#endif  // LQC_COMM_H_
