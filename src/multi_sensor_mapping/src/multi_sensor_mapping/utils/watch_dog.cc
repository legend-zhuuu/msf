#include "multi_sensor_mapping/utils/watch_dog.h"

namespace multi_sensor_mapping {

WatchDog::WatchDog() : start_flag_(false), max_wait_time_sec_(0.1) {}

WatchDog::WatchDog(double _max_wait_second)
    : start_flag_(false), max_wait_time_sec_(_max_wait_second) {}

void WatchDog::Feed() {
  mtx_.lock();
  start_ = std::chrono::system_clock::now();
  start_flag_ = true;
  mtx_.unlock();
}

void WatchDog::Reset() {
  mtx_.lock();
  start_flag_ = false;
  mtx_.unlock();
}

bool WatchDog::OK() {
  if (!start_flag_) {
    return true;
  }

  mtx_.lock();
  end_ = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end_ - start_;
  mtx_.unlock();

  if (elapsed_seconds.count() < 0) return false;
  return elapsed_seconds.count() < max_wait_time_sec_;
}

void WatchDog::SetMaxWaitTime(double _sec) { max_wait_time_sec_ = _sec; }

}  // namespace multi_sensor_mapping