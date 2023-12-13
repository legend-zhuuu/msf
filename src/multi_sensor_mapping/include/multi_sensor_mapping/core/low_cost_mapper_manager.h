#ifndef MSM_LOW_COST_MAPPER_MANAGER_H
#define MSM_LOW_COST_MAPPER_MANAGER_H

#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>
#include <thread>

#include "multi_sensor_mapping/backend/pose_graph_information.h"
#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/sync_queue.h"

#define ENABLE_3D_VISUALIZATION 1
#define ENABLE_LIDAR_TAG_DETECTION 1

namespace multi_sensor_mapping {

class FastLIOMapper;
class LidarMapSession;
class LidarMapper2DVisualizer;
class TinyPoseGraphProcessor;
class LidarMapperRosVisualizer;
class LidarDetector;

/**
 * @brief 建图状态
 *
 */
enum MappingStatus {
  ONLINE_ODOMETRY = 0,  // 在线里程计
  OFFLINE_MAPPING       // 离线建图
};

/**
 * @brief 低功耗建图管理器 (适用于ARM平台)
 *  分为在线里程计数据 + 关键帧采集 + 离线回环检测 + 离线回环优化
 *  可视化为二维图像可视化
 *
 */
class LowCostMapperManager {
 public:
  typedef std::unique_lock<std::mutex> Lock;

  /**
   * @brief Construct a new Low Cost Mapper Manager object
   *
   */
  LowCostMapperManager();

  /**
   * @brief Set the Param Set Path object 设置参数集合
   *
   * @param _param_path
   */
  void SetParamSetPath(std::string _param_path);

  /**
   * @brief Set the Cache Path object 设置缓存路径
   *
   * @param _cache_path
   */
  void SetCachePath(std::string _cache_path);

  /**
   * @brief Set the Pose Icon Path object 设置图标路径
   *
   * @param _icon_path
   */
  void SetPoseIconPath(std::string _icon_path);

  /**
   * @brief Set the Map Resolution object 设置地图路径
   *
   * @param _resolution
   */
  void SetMapResolution(float _resolution);

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
   * @brief 结束建图
   *
   */
  void StopMapping();

  /**
   * @brief 结束
   *
   */
  void Stop();

  /**
   * @brief 开始后端(测试用)
   *
   */
  void StartBackend(std::string _session_path);

 private:
  /**
   * @brief 激光建图处理
   *
   */
  void LidarMappingProcess();

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
   * @brief 处理后端进度
   *
   * @param _progress
   */
  void PutBackendProgress(const double& _progress);

  /**
   * @brief 处理位姿态图
   *
   * @param _graph
   */
  void PutPoseGraph(const PoseGraph& _graph);

  /**
   * @brief 获取局部匹配信息
   *
   * @param _local_match_info
   */
  void PutLocalMatchInfomation(const LocalMatchInformation& _local_match_info);

  /**
   * @brief 上传全局点云
   *
   * @param _cloud
   */
  void PutGlobalCloud(const CloudTypePtr& _cloud);

  /**
   * @brief 上传激光位姿态
   *
   * @param _tag_poses
   */
  void PutLidarTagPoses(const std::vector<PoseData>& _tag_poses);

  /**
   * @brief 关键帧选择
   *
   * @param _cloud
   * @param _pose
   */
  void KeyframeSelection(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 检查是否为关键帧
   *
   * @param _pose
   * @return true
   * @return false
   */
  bool CheckKeyScan(const PoseData& _pose);

  /**
   * @brief 发布局部匹配信息
   *
   * @param _local_match_info
   */
  void PublishLocalMatchInfo(const LocalMatchInformation& _local_match_info);

  /**
   * @brief 保存LidarTag姿态
   *
   */
  void SaveLidarTagPose();

 private:
  /// @brief ros节点
  ros::NodeHandle nh_;
  /// @brief 缓存路径
  std::string cache_path_;

  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;
  /// @brief tag初始化标志位
  bool tag_init_done_flag_;

  /// @brief tag姿态
  std::vector<PoseData> lidar_tag_pose_;

  /// @brief 建图状态
  MappingStatus mapping_status_;

  /// @brief 关键帧更新标志位
  std::atomic<bool> keyframe_update_flag_;
  /// @brief 关键帧阈值
  double key_pose_threshold_;

  /// @brief 参数路径
  std::string param_set_path_;

  /// @brief 激光建图
  std::shared_ptr<FastLIOMapper> lidar_mapper_;
  /// @brief 后端处理器
  std::shared_ptr<TinyPoseGraphProcessor> pose_graph_processor_;
  /// @brief 激光检测器
  std::shared_ptr<LidarDetector> lidar_detector_;
  /// @brief 激光建图任务
  std::shared_ptr<LidarMapSession> lidar_session_;
  /// @brief 激光建图可视化
  std::shared_ptr<LidarMapper2DVisualizer> visualizer_;

#if ENABLE_3D_VISUALIZATION
  /// @brief ROS可视化
  std::shared_ptr<LidarMapperRosVisualizer> ros_visualizer_;
#endif

  /// @brief 激光建图线程
  std::thread lidar_mapping_thread_;
  /// @brief 可视化线程
  std::thread visualization_thread_;

  /// @brief 激光位姿队列
  SyncQueue<PoseData> lidar_pose_queue_;
  /// @brief 激光位姿队列
  std::deque<PoseData> lidar_pose_cache_;

  /// @brief 普通帧队列
  SyncQueue<std::pair<CloudTypePtr, PoseData>> frame_queue_;
  /// @brief 后端进度队列
  SyncQueue<double> backend_progress_queue_;
  /// @brief 位姿图队列
  SyncQueue<PoseGraph> pose_graph_queue_;
  /// @brief 局部匹配信息队列
  SyncQueue<LocalMatchInformation> local_match_info_queue_;
  /// @brief 全局点云队列
  SyncQueue<CloudTypePtr> global_cloud_queue_;
  /// @brief 激光tag队列
  std::deque<std::vector<PoseData>> lidar_tag_queue_;

  /// @brief 关键帧点云
  CloudTypePtr key_frame_cloud_;
  /// @brief 关键帧位姿
  PoseData key_frame_pose_;
  /// @brief 姿态图标
  cv::Mat pose_icon_;

  /// @brief 锁
  std::mutex lidar_tag_mutex_;
  std::mutex lidar_pose_mutex_;
};

}  // namespace multi_sensor_mapping

#endif