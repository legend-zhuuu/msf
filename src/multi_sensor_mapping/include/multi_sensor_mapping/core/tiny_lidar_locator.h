#ifndef MSM_TINY_LIDAR_LOCATOR_H
#define MSM_TINY_LIDAR_LOCATOR_H

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <thread>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/sync_queue.h"

namespace multi_sensor_mapping {

class ParamSet;
class NdtScanMatcher;
class TinyLocationParams;

/**
 * @brief 定位状态
 *
 */
enum LocationStatus {
  LS_IDLE = 0,      // 空闲状态
  LS_INITIALIZING,  // 初始化中状态
  LS_NORMAL,        // 正常状态
  LS_ERROR          // 错误状态
};

/**
 * @brief 轻量级激光定位器
 *
 */
class TinyLidarLocator {
 public:
  typedef std::unique_lock<std::mutex> Lock;

  /**
   * @brief Construct a new Tiny Lidar Locator object
   *
   */
  TinyLidarLocator();

  /**
   * @brief Set the Params object 设置参数
   *
   * @param _param_set
   */
  void SetParams(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief Set the Map Path object 设置地图路径
   *
   * @param _map_path
   */
  void SetMapPath(std::string _map_path);

  /**
   * @brief Set the Map Cloud object 设置地图
   *
   * @param _map_cloud
   */
  void SetMapCloud(const CloudTypePtr& _map_cloud);

  /**
   * @brief  初始化
   *
   */
  void Init();

  /**
   * @brief 开始定位
   *
   */
  void Start();

  /**
   * @brief 结束定位
   *
   */
  void Stop();

  /**
   * @brief 输入初始化姿态
   *
   * @param _init_pose
   */
  void InputInitialPose(const PoseData& _init_pose);

  /**
   * @brief 输入LIO姿态
   *
   * @param _pose
   */
  void InputLIOPose(const PoseData& _pose);

  /**
   * @brief 输入关键帧
   *
   * @param _cloud
   * @param _pose
   */
  void InputKeyFrame(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 注册定位姿态回调函数
   *
   * @param _cb_location_pose
   */
  void RegLocationPose(
      const std::function<void(const PoseData&)>& _cb_location_pose);

  /**
   * @brief 注册定位点云回调函数
   *
   * @param _cb_location_cloud
   */
  void RegLocationCloud(
      const std::function<void(const CloudTypePtr&)>& _cb_location_cloud);

  /**
   * @brief 是否在运行
   *
   * @return true
   * @return false
   */
  bool IsRunning();

 private:
  /**
   * @brief 全局定位
   *
   */
  void GlobalLocationProcess();

  /**
   * @brief 融合处理
   *
   */
  void FusionProcess();

  /**
   * @brief 里程计姿态转换为定位姿态
   *
   * @param _odom_pose_data
   * @return Eigen::Matrix4d
   */
  Eigen::Matrix4d OdometryPose2LocationPose(const PoseData& _odom_pose_data);

  /**
   * @brief 重置定位器
   *
   */
  void ResetLocator();

 private:
  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;

  /// @brief 定位状态
  LocationStatus location_status_;
  /// @brief 更新地图标志位
  std::atomic<bool> update_map_flag_;
  /// @brief 更新初始化位姿标志位
  std::atomic<bool> update_init_pose_flag_;
  /// @brief 更新关键帧标志位
  std::atomic<bool> update_key_frame_flag_;
  /// @brief 更新关键帧标志位
  std::atomic<bool> update_key_frame_pose_flag_;
  /// @brief 更新lio位姿标志位
  std::atomic<bool> update_lio_pose_flag_;

  /// @brief 地图点云
  CloudTypePtr map_cloud_;
  /// @brief 初始化姿态
  PoseData initial_pose_;
  /// @brief 最新的lio姿态
  PoseData laetst_lio_pose_;
  /// @brief 最新关键帧位姿
  PoseData latest_key_frame_pose_;
  /// @brief 最新关键帧点云
  CloudTypePtr latest_key_frame_cloud_;

  /// @brief 关键帧姿态偏置
  Eigen::Matrix4d key_frame_pose_bias_;

  /// @brief 锁
  std::mutex lio_pose_mutex_;
  std::mutex key_frame_pose_mutex_;

  /// @brief 定位参数
  std::shared_ptr<TinyLocationParams> location_params_;
  /// @brief 点云配准器
  std::shared_ptr<NdtScanMatcher> scan_matcher_;

  /// @brief 全局定位线程
  std::thread global_location_thread_;
  /// @brief 融合线程
  std::thread fusion_thread_;

  /// @brief 定位姿态回调函数
  std::function<void(const PoseData&)> cb_put_location_pose_;
  /// @brief 定位点云回调函数
  std::function<void(const CloudTypePtr&)> cb_put_location_cloud_;
};
}  // namespace multi_sensor_mapping

#endif