#ifndef LQC_UTILS_TYPES_H_
#define LQC_UTILS_TYPES_H_

#include <mutex>
#include <Eigen/Dense>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

namespace lqc {
using LockGuard = std::lock_guard<std::mutex>;
using Arr3 = Eigen::Array3f;
using Arr4 = Eigen::Array4f;
using ArrX = Eigen::ArrayXf;
template<int N, typename T = float>
using Arr = Eigen::Array<T, N, 1>;

using Vec3 = Eigen::Vector3f;
using Vec4 = Eigen::Vector4f;
using VecX = Eigen::VectorXf;
template<int N, typename T = float>
using Vec = Eigen::Matrix<T, N, 1>;

using Mat3 = Eigen::Matrix3f;
using MatX = Eigen::MatrixXf;
template<int N, typename T = float>
using Mat = Eigen::Matrix<T, N, N>;

#define DEFINE_REF_ALIASES(type)                   \
  using r##type = Eigen::Ref<type>;                \
  using c##type = const Eigen::Ref<const type> &;

DEFINE_REF_ALIASES(Arr3)
DEFINE_REF_ALIASES(ArrX)
DEFINE_REF_ALIASES(Vec3)
DEFINE_REF_ALIASES(Vec4)
DEFINE_REF_ALIASES(VecX)
DEFINE_REF_ALIASES(Mat3)
DEFINE_REF_ALIASES(MatX)

#undef DEFINE_REF_ALIASES

namespace sdk = UNITREE_LEGGED_SDK;


/// Simple timer implementation
namespace timer {
using namespace std::chrono;
using clock = high_resolution_clock;
using ms = milliseconds;
using us = microseconds;
using ns = nanoseconds;

class Timer {
 public:
  void start() { start_ = clock::now(); }
  void stop() {
    stop_ = clock::now();
    count_ += 1;
    duration_ += stop_ - start_;
  }
  void clear() {
    duration_ = clock::duration::zero();
    count_ = 0;
  }

  std::size_t count() const { return count_; }
  clock::duration total() const { return duration_; }
  clock::duration mean() const {
    if (!count_) return clock::duration{0};
    return duration_ / count_;
  }

  template<class T>
  std::size_t total() {
    return duration_cast<T>(total()).count();
  }
  template<class T>
  std::size_t mean() {
    return duration_cast<T>(mean()).count();
  }

 private:
  clock::time_point start_, stop_;
  clock::duration duration_{0};
  std::size_t count_ = 0;
};

class TimerGuard {
 public:
  explicit TimerGuard(Timer &timer) : timer_{timer} { timer_.start(); }
  ~TimerGuard() { timer_.stop(); }

 private:
  Timer &timer_;
};

}  // namespace timer
}  // namespace lqc

#endif  // LQC_UTILS_TYPES_H_
