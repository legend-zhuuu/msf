#ifndef MSM_SDK_LIDAR_IMU_MAPPING_H
#define MSM_SDK_LIDAR_IMU_MAPPING_H

#include <functional>
#include <memory>
#include <thread>

#include "msm_types.h"

namespace multi_sensor_mapping {
class LidarImuMapper;
class LidarMapSession;
}  // namespace multi_sensor_mapping

namespace msm_sdk {

/**
 * @brief 激光-IMU建图API
 *
 */
class APILidarImuMapping {
 public:
  /**
   * @brief Construct a new APILidarImuMapping object 构造函数
   *
   */
  APILidarImuMapping();

  /**
   * @brief Set the Mapping Param object 设置建图参数
   *
   * @param _scene
   * @param use_loop_closure
   */
  void SetMappingParam(MappingScene _scene, bool use_loop_closure);

  /**
   * @brief 初始化
   *
   */
  bool Init(std::string _param_path);

  /**
   * @brief 开始建图
   *
   * @param _bag_path
   */
  void Start(std::string _bag_path);

  /**
   * @brief 停止建图
   *
   */
  void Stop();

  /**
   * @brief 保存地图
   *
   * @param _save_path
   */
  void SaveMapSession(std::string _save_path = "");

  /**
   * @brief 获取地图指针
   *
   * @return std::shared_ptr<multi_sensor_mapping::LidarMapSession>
   */
  std::shared_ptr<multi_sensor_mapping::LidarMapSession> GetLidarMapSession();

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

  /**
   * @brief 注册建图进度回调函数
   *
   * @param _cb_progress
   */
  void RegMappingProgressCallback(
      const std::function<void(const double)>& _cb_progress);

 private:
  /**
   * @brief 核心处理函数
   *
   */
  void Process();

 private:
  /// @brief 激光建图
  std::shared_ptr<multi_sensor_mapping::LidarImuMapper> mapper_;

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
  /// @brief 建图进度回调函数
  std::function<void(const double)> cb_progress_;
};

}  // namespace msm_sdk

#endif