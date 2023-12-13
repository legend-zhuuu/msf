
#ifndef MSM_LIDAR_MAPPER_ROS_VISUALIZER_H
#define MSM_LIDAR_MAPPER_ROS_VISUALIZER_H

#include <tf/transform_broadcaster.h>

#include <mutex>
#include <thread>

#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "multi_sensor_mapping/visualizer/pose_visualizer.h"

namespace multi_sensor_mapping {

class ParamSet;
class ExtrinsicParams;
class SensorParams;
class PoseVisualizer;
class PosePathVisualizer;
class PointCloudVisualizer;
class PoseGraphVisualizer;
class PoseGraph;
class PoseOdomVisualizer;
class SelectedEdgeVisualizer;
class SurfelMapVisualizer;
struct SurfelUnit;
class ReflectionColumnVisualizer;
struct ReflectionColumn;
class PoseArrayVisualizer;
struct PoseData;

/**
 * @brief The LidarMapperRosVisualizer class 激光建图可视化管理器
 */
class LidarMapperRosVisualizer {
 public:
  typedef std::shared_ptr<LidarMapperRosVisualizer> Ptr;

  /**
   * @brief LidarMapperRosVisualizer 构造函数
   * @param _nh
   */
  LidarMapperRosVisualizer();

  /**
   * @brief LidarMapperRosVisualizer 构造函数
   * @param _nh
   * @param _param_set
   */
  LidarMapperRosVisualizer(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief DisplayCurrentPose 显示当前姿态
   * @param _current_pose
   */
  void DisplayCurrentPose(const Eigen::Matrix4d& _current_pose);

  /**
   * @brief DisplayCurrentOdom 显示当前里程计
   * @param _current_pose
   */
  void DisplayCurrentOdom(const double timestamp,
                          const Eigen::Matrix4d& _current_pose);

  /**
   * @brief DisplayCurrentCloud 显示当前点云
   * @param _current_cloud
   */
  void DisplayCurrentCloud(CloudTypePtr& _current_cloud);

  /**
   * @brief DisplayGlobalCloud 显示全局点云
   * @param _global_cloud
   */
  void DisplayGlobalCloud(CloudTypePtr& _global_cloud);

  /**
   * @brief 显示pose graph
   *
   * @param _graph
   */
  void DisplayPoseGraph(const PoseGraph& _graph);

  /**
   * @brief 显示闭环点云
   *
   * @param _target_cloud
   * @param _source_cloud
   */
  void DisplayLoopClosureCloud(const CloudTypePtr& _target_cloud,
                               const CloudTypePtr& _source_cloud);

  /**
   * @brief 显示选择的边
   *
   * @param _pos1
   * @param _pos2
   */
  void DisplaySelectedEdge(const Eigen::Vector3d& _pos1,
                           const Eigen::Vector3d& _pos2);

  /**
   * @brief 显示面元地图
   *
   * @param _surfel_units
   */
  void DisplaySurfelMap(const std::vector<SurfelUnit>& _surfel_units);

  /**
   * @brief 显示反光柱
   *
   * @param _reflection_columns
   */
  void DisplayReflectionColumn(
      const std::vector<ReflectionColumn>& _reflection_columns);

  /**
   * @brief 显示tag姿态
   *
   * @param _pose_vec
   */
  void DisplayTagPoses(const std::vector<PoseData>& _pose_vec);

 private:
  /**
   * @brief InitTfTree 初始化静态TF
   */
  void InitTfTree();

  /**
   * @brief InitVisualizers 初始化可视化工具
   */
  void InitVisualizers();

  /**
   * @brief PubExtrinsicTf 发布外参TF tree
   */
  void PubExtrinsicTf();

  /**
   * @brief PubBaselinkTf 发布Baselink的tf
   * @param _map_to_baselink
   * @param _stamp
   */
  void PubBaselinkTf(const Eigen::Matrix4d& _map_to_baselink, ros::Time _stamp);

 private:
  /// @brief ros节点
  ros::NodeHandle nh_;

  /// @brief 当前姿态显示器
  std::shared_ptr<PoseVisualizer> current_pose_visualizer_;
  /// @brief 当前点云显示器
  std::shared_ptr<PointCloudVisualizer> current_cloud_visualizer_;
  /// @brief 全局地图显示器
  std::shared_ptr<PointCloudVisualizer> global_map_visualizer_;
  /// @brief 闭环target点云显示器
  std::shared_ptr<PointCloudVisualizer> loop_closure_target_cloud_visualizer_;
  /// @brief 闭环source点云显示器
  std::shared_ptr<PointCloudVisualizer> loop_closure_source_cloud_visualizer_;

  /// @brief 姿态路径显示器
  std::shared_ptr<PosePathVisualizer> pose_path_visualizer_;
  /// @brief 里程计显示器
  std::shared_ptr<PoseOdomVisualizer> pose_odom_visualizer_;
  /// @brief 因子图显示器
  std::shared_ptr<PoseGraphVisualizer> graph_visualizer_;
  /// @brief 被选择的边显示器
  std::shared_ptr<SelectedEdgeVisualizer> selected_edge_visualizer_;
  /// @brief surfel map显示器
  std::shared_ptr<SurfelMapVisualizer> surfel_map_visualizer_;
  /// @brief 反光柱显示器
  std::shared_ptr<ReflectionColumnVisualizer> reflection_column_visualizer_;
  /// @brief tag 姿态
  std::shared_ptr<PoseArrayVisualizer> tag_pose_array_visualizer_;

  /// @brief 外参参数
  std::shared_ptr<ExtrinsicParams> extrinsic_params_;
  /// @brief 传感器参数
  std::shared_ptr<SensorParams> sensor_params_;
  /// @brief 传感器外参tf
  std::vector<tf::StampedTransform> sensor_tf_vec_;
};

}  // namespace multi_sensor_mapping

#endif
