#ifndef MSM_SDK_LIDAR_IMU_ODOMETRY_H
#define MSM_SDK_LIDAR_IMU_ODOMETRY_H

#include <functional>
#include <memory>
#include <thread>

#include "msm_types.h"

namespace multi_sensor_mapping {
class FastLIOMapper;
}

namespace msm_sdk {

/**
 * @brief 激光-IMU里程计API
 *
 */
class APILidarImuOdometry {
 public:
  /**
   * @brief Construct a new APILidarImuOdometry object
   *
   */
  APILidarImuOdometry();

  /**
   * @brief 初始化
   *
   */
  bool Init(std::string _param_path);

  /**
   * @brief 开始
   *
   */
  void Start();

  /**
   * @brief 结束
   *
   */
  void Stop();

  /**
   * @brief 注册激光姿态回调函数
   *
   * @param _cb_pose
   */
  void RegLidarPoseCallback(
      const std::function<void(const StampedPose&)>& _cb_pose);

  /**
   * @brief 注册激光点云回调函数
   *
   * @param _cb_cloud
   */
  void RegFrameCloudCallback(
      const std::function<void(const StampedCloud&)>& _cb_cloud);

 private:
  /**
   * @brief 核心处理函数
   *
   */
  void Process();

 private:
  std::shared_ptr<multi_sensor_mapping::FastLIOMapper> mapper_;

  /// @brief 退出线程标志位
  bool exit_process_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;

  /// @brief 处理线程
  std::thread process_thead_;

  /// @brief  激光姿态回调函数
  std::function<void(const StampedPose&)> cb_lidar_pose_;
  /// @brief 激光点云回调函数
  std::function<void(const StampedCloud&)> cb_frame_cloud_;
};
}  // namespace msm_sdk

#endif