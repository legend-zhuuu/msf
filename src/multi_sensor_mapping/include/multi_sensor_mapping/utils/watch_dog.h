#ifndef MSM_WATCH_DOG_H
#define MSM_WATCH_DOG_H

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <mutex>

namespace multi_sensor_mapping {
/**
 * @brief 看门狗
 *
 */
class WatchDog {
 public:
  typedef std::shared_ptr<WatchDog> Ptr;

  /**
   * @brief Construct a new Watch Dog object
   *
   */
  WatchDog();

  /**
   * @brief Construct a new Watch Dog object
   *
   * @param _max_wait_second
   */
  WatchDog(double _max_wait_second);

  /**
   * @brief 喂狗
   *
   */
  void Feed();

  /**
   * @brief 重置
   *
   */
  void Reset();

  /**
   * @brief 检测是否OK
   *
   * @return true
   * @return false
   */
  bool OK();

  /**
   * @brief 设置最大等待时间
   *
   * @param _sec
   */
  void SetMaxWaitTime(double _sec);

 private:
  /// @brief 时间
  std::chrono::time_point<std::chrono::system_clock> start_, end_;
  /// @brief 开启标志位
  bool start_flag_;
  /// @brief 最大等待时间
  double max_wait_time_sec_;
  /// @brief 锁
  std::mutex mtx_;
};

}  // namespace multi_sensor_mapping
#endif