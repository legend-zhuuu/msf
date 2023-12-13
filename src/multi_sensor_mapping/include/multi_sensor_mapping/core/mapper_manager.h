#ifndef MSM_MAPPER_MANAGER_H
#define MSM_MAPPER_MANAGER_H

#include <ros/ros.h>

#include <memory>
#include <thread>

#include "multi_sensor_mapping/backend_panel_cmd.h"
#include "multi_sensor_mapping/core/lidar_mapper_base.h"
#include "multi_sensor_mapping/core/pose_graph_processor.h"
#include "multi_sensor_mapping/utils/sync_queue.h"
#include "multi_sensor_mapping/utils/utils_common.h"

namespace multi_sensor_mapping {

class LidarMapperRosVisualizer;
class LidarMapSession;

/**
 * @brief 建图管理器
 *
 */
class MapperManager {
 public:
  typedef std::shared_ptr<MapperManager> Ptr;

  /**
   * @brief Construct a new Mapper Manager object 构造函数
   *
   */
  MapperManager();

  /**
   * @brief Set the Param Set Path object 设置参数集合
   *
   * @param _param_path
   */
  void SetParamSetPath(std::string _param_path);

  /**
   * @brief 设置数据包路径
   *
   * @param _bag_path
   */
  void SetBagPath(std::string _bag_path, double _start_time = 0,
                  double _map_durr = -1);

  /**
   * @brief Set the Session Path object 设置任务路径
   *
   * @param session_path
   */
  void SetSessionPath(std::string session_path);

  /**
   * @brief 开始自动闭环检测
   *
   */
  void StartAutoLoopClosureDetection();

  /**
   * @brief 初始化
   *
   */
  void Init();

  /**
   * @brief 开始建图
   *
   */
  void Start();

  /**
   * @brief 开始前端建图
   *
   */
  void StartFrontend();

  /**
   * @brief 开始后端优化
   *
   */
  void StartBackend();

  /**
   * @brief 结束定位
   *
   */
  void Stop();

  /**
   * @brief 保存地图
   *
   */
  void SaveActiveMapSession();

 private:
  /**
   * @brief 注册ROS订阅器
   *
   */
  void RigisterSub();

  /**
   * @brief 后端命令回调函数
   *
   * @param _panel_cmd
   */
  void BackendPanelCmdHandler(
      const multi_sensor_mapping::backend_panel_cmd::ConstPtr& _panel_cmd);

  /**
   * @brief 交互处理
   *
   */
  void InteractiveProcess();

  /**
   * @brief 可视化渲染处理
   *
   */
  void GlobalMapVisualizationProcess();

  /**
   * @brief 处理异常信息
   *
   * @param _error_msg
   */
  void PutMapperException(const MSMError& _error_msg);

  /**
   * @brief 处理激光姿态
   *
   * @param _pose_data
   */
  void PutLidarPose(const PoseData& _pose_data);

  /**
   * @brief 处理关键帧
   *
   * @param _cloud
   * @param _pose
   */
  void PutKeyFrame(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 处理普通帧
   *
   * @param _cloud
   * @param _pose
   */
  void PutFrame(const CloudTypePtr& _cloud, const PoseData& _pose);

  /**
   * @brief 处理建图进度
   *
   * @param _progress
   */
  void PutMappingProgress(const double& _progress);

  /**
   * @brief 处理Pose graph异常
   *
   * @param _error_msg
   */
  void PutPoseGraphException(const MSMError& _error_msg);

  /**
   * @brief 处理Pose graph
   *
   * @param _graph
   */
  void PutPoseGraph(const PoseGraph& _graph);

  /**
   * @brief 处理全局地图
   *
   * @param _cloud
   */
  void PutGlobalCloud(const CloudTypePtr& _cloud);

  /**
   * @brief 获取局部匹配信息
   *
   * @param _local_match_info
   */
  void PutLocalMatchInfomation(const LocalMatchInformation& _local_match_info);

  /**
   * @brief 获取地图任务
   *
   * @param _session
   */
  void PutMapSession(const std::shared_ptr<LidarMapSession>& _session);

  /**
   * @brief 发布错误信息
   *
   * @param _error
   */
  void PublishErrorInfo(MSMError _error);

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

  /**
   * @brief 发布关键帧信息
   *
   * @param _cloud
   * @param _pose
   */
  void PublishKeyFrame(CloudTypePtr _cloud, PoseData _pose);

  /**
   * @brief 发布建图进度
   *
   * @param _progress
   */
  void PublishMappingProgress(double _progress);

  /**
   * @brief 发布位姿图
   *
   * @param _graph
   */
  void PublishPoseGraph(const PoseGraph& _graph);

  /**
   * @brief 发布局部匹配信息
   *
   * @param _local_match_info
   */
  void PublishLocalMatchInfo(const LocalMatchInformation& _local_match_info);

 private:
  /// @brief ros节点
  ros::NodeHandle nh_;
  /// @brief 后端Panel命令订阅器
  ros::Subscriber sub_backend_panel_cmd_;
  /// @brief 后端信息发布器
  ros::Publisher pub_backend_info_;

  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;

  /// @brief 参数路径
  std::string param_set_path_;
  /// @brief 地图数据包
  std::string bag_path_;
  /// @brief 数据包起始时间
  double bag_start_time_;
  /// @brief 数据包长度
  double bag_durr_;
  /// @brief 建图模式
  MappingMode mapping_mode_;
  /// @brief 激光建图
  std::shared_ptr<LidarMapperBase> lidar_mapper_;
  /// @brief 后端处理器
  std::shared_ptr<PoseGraphProcessor> pose_graph_processor_;
  /// @brief 可视化器
  std::shared_ptr<LidarMapperRosVisualizer> visualizer_;

  /// @brief 激光建图任务
  std::shared_ptr<LidarMapSession> active_session_;
  /// @brief 任务更新标志位
  bool lidar_map_session_update_flag_;

  /// @brief 交互线程
  std::thread interactive_thread_;
  /// @brief 可视化渲染线程
  std::thread visualization_thread_;

  /// @brief 建图信息队列
  SyncQueue<MSMError> mapper_error_queue_;
  /// @brief 激光位姿队列
  SyncQueue<PoseData> lidar_pose_queue_;
  /// @brief 关键帧队列
  SyncQueue<std::pair<CloudTypePtr, PoseData>> key_frame_queue_;
  /// @brief 普通帧队列
  SyncQueue<std::pair<CloudTypePtr, PoseData>> frame_queue_;
  /// @brief 建图进度队列
  SyncQueue<double> mapping_progress_queue_;
  /// @brief 位姿图队列
  SyncQueue<PoseGraph> pose_graph_queue_;
  /// @brief 全局地图队列
  SyncQueue<CloudTypePtr> global_cloud_queue_;
  /// @brief 局部匹配信息队列
  SyncQueue<LocalMatchInformation> local_match_info_queue_;
};

}  // namespace multi_sensor_mapping

#endif