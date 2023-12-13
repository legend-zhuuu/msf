#ifndef MSM_UTILS_SYNC_QUEUE_H
#define MSM_UTILS_SYNC_QUEUE_H

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

namespace multi_sensor_mapping {

/**
 * @brief 同步队列
 *
 * @tparam T
 */
template <typename T>
class SyncQueue {
 public:
  /**
   * @brief 数据入栈
   *
   * @param _data
   */
  inline void Push(const T& _data) {
    std::lock_guard<std::mutex> lg(mtx_);
    data_queue_.push(_data);
  }

  /**
   * @brief 数据出栈
   *
   * @return T
   */
  inline T Pop() {
    T value;

    std::lock_guard<std::mutex> lg(mtx_);
    if (!data_queue_.empty()) {
      value = data_queue_.front();
      data_queue_.pop();
    }

    return value;
  }

  /**
   * @brief 最新数据出栈
   *
   * @return T
   */
  inline T PopLatest() {
    T value;
    std::lock_guard<std::mutex> lg(mtx_);
    while (!data_queue_.empty()) {
      value = data_queue_.front();
      data_queue_.pop();
    }

    return value;
  }

  /**
   * @brief 是否为空
   *
   * @return true
   * @return false
   */
  bool Empty() { return data_queue_.empty(); }

  /**
   * @brief 清空队列
   *
   */
  void Clear() {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lg(mtx_);
    swap(empty, data_queue_);
  }

  /**
   * @brief 数量
   *
   * @return size_t
   */
  size_t Size() { return data_queue_.size(); }

 private:
  /// @brief 锁
  std::mutex mtx_;
  /// @brief 数据队列
  std::queue<T> data_queue_;
};
}  // namespace multi_sensor_mapping

#endif