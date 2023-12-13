#ifndef MSM_LOW_COST_LOCATOR_MANAGER_H
#define MSM_LOW_COST_LOCATOR_MANAGER_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <memory>
#include <thread>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/sync_queue.h"

/// @brief 使能3D可视化
#define ENABLE_3D_VISUALIZATION 1
#define ENABLE_2D_VISUALIZATION 0

namespace multi_sensor_mapping {

class FastLIOMapper;
class LidarMapper2DVisualizer;
class TinyLidarLocator;
class LidarMapperRosVisualizer;
class LidarDetector;

/**
 * @brief 低功耗定位管理器 (适用于ARM平台)
 * 分为在线里程计 (高频) + 全局定位配准 (低频)
 * 可视化为二维图像可视化
 *
 */
class LowCostLocatorManager {
 public:
  /**
   * @brief Construct a new Low Cost Locator Manager object
   *
   */
  LowCostLocatorManager();

  /**
   * @brief Set the Param Set Path object 设置参数集合
   *
   * @param _param_path
   */
  void SetParamSetPath(std::string _param_path);

  /**
   * @brief Set the Map Path object 设置地图路径
   *
   * @param _map_path
   */
  void SetMapPath(std::string _map_path);

  /**
   * @brief Set the Map Resolution object 设置地图路径
   *
   * @param _resolution
   */
  void SetMapResolution(float _resolution);

  /**
   * @brief Set the Record Path object 设置记录路径
   *
   * @param _record_path
   */
  void SetRecordPath(std::string _record_path);

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
   * @brief 定位处理
   *
   */
  void LocationProcess();

  /**
   * @brief 可视化处理
   *
   */
  void VisualizationProcess();

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
   * @brief 处理定位姿态
   *
   * @param _pose_data
   */
  void PutLocationPose(const PoseData& _pose_data);

  /**
   * @brief 处理定位点云(显示)
   *
   * @param _cloud
   */
  void PutLocationCloud(const CloudTypePtr& _cloud);

  /**
   * @brief 上传激光位姿态
   *
   * @param _tag_poses
   */
  void PutLidarTagPoses(const std::vector<PoseData>& _tag_poses);

  /**
   * @brief InitPoseHandler 初始化位姿回调函数
   * @param msg
   */
  void InitPoseHandler(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _msg);

  /**
   * @brief SaveCacheData 保存缓存数据
   *
   */
  void SaveCacheData();

 private:
  /// @brief ros节点
  std::unique_ptr<ros::NodeHandle> nh_;
  /// @brief 初始化姿态订阅器
  ros::Subscriber sub_initial_pose_;

  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief tag初始化标志位
  bool tag_init_done_flag_;
  /// @brief 录制姿态标志位
  bool record_pose_flag_;
  /// @brief 录制缓存路径
  std::string record_cache_path_;

  /// @brief 参考tag姿态
  std::vector<Eigen::Matrix4d> ref_lidar_tag_pose_;

  /// @brief 地图路径
  std::string map_path_;
  /// @brief 参数路径
  std::string param_set_path_;

  /// @brief 全局定位时间间隔
  double global_registration_interval_;
  /// @brief 自启动标志位
  bool auto_start_flag_;
  /// @brief 全局地图
  CloudTypePtr global_map_;

  /// @brief 激光建图
  std::shared_ptr<FastLIOMapper> lidar_mapper_;
  /// @brief 激光定位
  std::shared_ptr<TinyLidarLocator> lidar_locator_;
  /// @brief 激光检测器
  std::shared_ptr<LidarDetector> lidar_detector_;
  /// @brief 激光建图可视化
  std::shared_ptr<LidarMapper2DVisualizer> visualizer_;
#if ENABLE_3D_VISUALIZATION
  /// @brief ROS可视化
  std::shared_ptr<LidarMapperRosVisualizer> ros_visualizer_;
#endif

  /// @brief 定位线程
  std::thread location_thread_;
  /// @brief 可视化线程
  std::thread visualization_thread_;

  /// @brief 激光位姿队列
  SyncQueue<PoseData> lidar_pose_queue_;
  /// @brief 普通帧队列
  SyncQueue<std::pair<CloudTypePtr, PoseData>> frame_queue_;
  /// @brief 定位姿态队列
  SyncQueue<PoseData> location_pose_queue_;
  /// @brief 定位点云队列
  SyncQueue<CloudTypePtr> location_cloud_queue_;
  /// @brief 激光tag队列
  SyncQueue<std::vector<PoseData>> lidar_tag_queue_;

  /// @brief 激光位姿缓存
  std::vector<PoseData> location_pose_cache_container_;
};

}  // namespace multi_sensor_mapping

#endif
