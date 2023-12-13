#ifndef MSM_ODOMETRY_MANAGER_H
#define MSM_ODOMETRY_MANAGER_H

#include <ros/ros.h>

#include <memory>
#include <thread>

#include "multi_sensor_mapping/core/lidar_mapper_base.h"
#include "multi_sensor_mapping/utils/sync_queue.h"
#include "multi_sensor_mapping/utils/utils_common.h"

namespace multi_sensor_mapping {

class LidarMapperRosVisualizer;

/**
 * @brief 里程计管理器
 *
 */
class OdometryManager {
 public:
  /**
   * @brief Construct a new Odometry Manager object
   *
   */
  OdometryManager();

  /**
   * @brief Set the Param Set Path object 设置参数集合
   *
   * @param _param_path
   */
  void SetParamSetPath(std::string _param_path);

  /**
   * @brief 初始化
   *
   */
  void Init();

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

 private:
  /**
   * @brief 交互处理
   *
   */
  void InteractiveProcess();

  /**
   * @brief 处理激光姿态
   *
   * @param _pose_data
   */
  void PutLidarPose(const PoseData& _pose_data);

  /**
   * @brief 处理普通帧
   *
   * @param _cloud
   * @param _pose
   */
  void PutFrame(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 发布激光姿态
   *
   * @param _pose_data
   */
  void PublishLidarPose(PoseData _pose_data);

  /**
   * @brief 发布普通帧点云
   *
   * @param _cloud
   * @param _pose
   */
  void PublishFrame(CloudTypePtr _cloud, PoseData _pose);

 private:
  /// @brief ros节点
  ros::NodeHandle nh_;

  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;

  /// @brief 参数路径
  std::string param_set_path_;

  /// @brief 激光建图
  std::shared_ptr<LidarMapperBase> lidar_mapper_;
  /// @brief 可视化器
  std::shared_ptr<LidarMapperRosVisualizer> visualizer_;

  /// @brief 交互线程
  std::thread interactive_thread_;

  /// @brief 建图信息队列
  SyncQueue<MSMError> mapper_error_queue_;
  /// @brief 激光位姿队列
  SyncQueue<PoseData> lidar_pose_queue_;
  /// @brief 普通帧队列
  SyncQueue<std::pair<CloudTypePtr, PoseData>> frame_queue_;
};
}  // namespace multi_sensor_mapping

#endif