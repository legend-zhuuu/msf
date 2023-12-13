#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/param_set.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/visualizer/odom_visualizer.h"
#include "multi_sensor_mapping/visualizer/path_visualizer.h"
#include "multi_sensor_mapping/visualizer/pointcloud_visualizer.h"
#include "multi_sensor_mapping/visualizer/pose_array_visualizer.h"
#include "multi_sensor_mapping/visualizer/pose_graph_visualizer.h"
#include "multi_sensor_mapping/visualizer/pose_visualizer.h"
#include "multi_sensor_mapping/visualizer/reflection_column_visualizer.h"
#include "multi_sensor_mapping/visualizer/selected_edge_visualizer.h"
#include "multi_sensor_mapping/visualizer/surfel_map_visualizer.h"

namespace multi_sensor_mapping {

LidarMapperRosVisualizer::LidarMapperRosVisualizer() { InitVisualizers(); }

LidarMapperRosVisualizer::LidarMapperRosVisualizer(
    const std::shared_ptr<ParamSet> &_param_set) {
  sensor_params_ = _param_set->GetSensorParams();
  extrinsic_params_ = _param_set->GetExtrinsicParams();
  InitTfTree();
  InitVisualizers();
  PubExtrinsicTf();
}

void LidarMapperRosVisualizer::DisplayCurrentPose(
    const Eigen::Matrix4d &_current_pose) {
  Eigen::Matrix4d t_m2l = _current_pose;
  Eigen::Matrix4d t_b2l =
      extrinsic_params_->lidar_to_baselink[sensor_params_->major_lidar_];
  Eigen::Matrix4d t_m2b = t_m2l * t_b2l.inverse();

  PubBaselinkTf(t_m2b, ros::Time::now());
  PubExtrinsicTf();
  current_pose_visualizer_->Display(_current_pose);
  pose_path_visualizer_->Display(_current_pose);
}

void LidarMapperRosVisualizer::DisplayCurrentOdom(
    const double timestamp, const Eigen::Matrix4d &_current_pose) {
  pose_odom_visualizer_->Display(_current_pose);
}

void LidarMapperRosVisualizer::DisplayCurrentCloud(
    CloudTypePtr &_current_cloud) {
  CloudTypePtr cloud_low_res(new CloudType());
  pcl::copyPointCloud(*_current_cloud, *cloud_low_res);
  utils::DownsampleCloud(*cloud_low_res, 0.1);
  current_cloud_visualizer_->Display(cloud_low_res);
}

void LidarMapperRosVisualizer::DisplayGlobalCloud(CloudTypePtr &_global_cloud) {
  CloudTypePtr cloud_low_res(new CloudType());
  utils::DownsampleCloudAdapted(*_global_cloud, *cloud_low_res, 0.3);
  global_map_visualizer_->Display(cloud_low_res);
}

void LidarMapperRosVisualizer::DisplayPoseGraph(const PoseGraph &_graph) {
  graph_visualizer_->Display(_graph);
}

void LidarMapperRosVisualizer::DisplayLoopClosureCloud(
    const CloudTypePtr &_target_cloud, const CloudTypePtr &_source_cloud) {
  CloudTypePtr target_cloud_low_res(new CloudType());
  pcl::copyPointCloud(*_target_cloud, *target_cloud_low_res);
  utils::DownsampleCloud(*target_cloud_low_res, 0.1);

  CloudTypePtr source_cloud_low_res(new CloudType());
  pcl::copyPointCloud(*_source_cloud, *source_cloud_low_res);
  utils::DownsampleCloud(*source_cloud_low_res, 0.1);

  loop_closure_target_cloud_visualizer_->Display(target_cloud_low_res);
  loop_closure_source_cloud_visualizer_->Display(source_cloud_low_res);
}

void LidarMapperRosVisualizer::DisplaySelectedEdge(
    const Eigen::Vector3d &_pos1, const Eigen::Vector3d &_pos2) {
  selected_edge_visualizer_->Display(_pos1, _pos2);
}

void LidarMapperRosVisualizer::DisplaySurfelMap(
    const std::vector<SurfelUnit> &_surfel_units) {
  surfel_map_visualizer_->Display(_surfel_units);
}

void LidarMapperRosVisualizer::DisplayReflectionColumn(
    const std::vector<ReflectionColumn> &_reflection_columns) {
  reflection_column_visualizer_->Display(_reflection_columns);
}

void LidarMapperRosVisualizer::DisplayTagPoses(
    const std::vector<PoseData> &_pose_vec) {
  tag_pose_array_visualizer_->Display(_pose_vec);
}

void LidarMapperRosVisualizer::InitTfTree() {
  if (!sensor_params_->lidar_param.empty()) {
    tf::Transform t = utils::TransdToTransform(
        extrinsic_params_->lidar_to_baselink[sensor_params_->major_lidar_]);
    sensor_tf_vec_.push_back(
        tf::StampedTransform(t, ros::Time::now(), "base_link", "lidar"));
  }

  if (sensor_params_->imu_param.use_flag) {
    tf::Transform t =
        utils::TransdToTransform(extrinsic_params_->imu_to_baselink);
    sensor_tf_vec_.push_back(
        tf::StampedTransform(t, ros::Time::now(), "base_link", "imu"));
  }

  if (sensor_params_->wheel_param.use_flag) {
    tf::Transform t =
        utils::TransdToTransform(extrinsic_params_->wheel_to_baselink);
    sensor_tf_vec_.push_back(
        tf::StampedTransform(t, ros::Time::now(), "base_link", "odom"));
  }
}

void LidarMapperRosVisualizer::InitVisualizers() {
  current_pose_visualizer_ =
      std::make_shared<PoseVisualizer>(nh_, "map", "lidar_pose");

  current_cloud_visualizer_ =
      std::make_shared<PointCloudVisualizer>(nh_, "map", "current_cloud");
  global_map_visualizer_ =
      std::make_shared<PointCloudVisualizer>(nh_, "map", "global_map");

  pose_path_visualizer_ =
      std::make_shared<PosePathVisualizer>(nh_, "map", "lidar_path");
  pose_odom_visualizer_ =
      std::make_shared<PoseOdomVisualizer>(nh_, "map", "lidar_odom");
  graph_visualizer_ =
      std::make_shared<PoseGraphVisualizer>(nh_, "map", "factor_graph");
  selected_edge_visualizer_ =
      std::make_shared<SelectedEdgeVisualizer>(nh_, "map", "selected_edge");

  loop_closure_target_cloud_visualizer_ =
      std::make_shared<PointCloudVisualizer>(nh_, "map", "lc_target_cloud");
  loop_closure_source_cloud_visualizer_ =
      std::make_shared<PointCloudVisualizer>(nh_, "map", "lc_source_cloud");

  surfel_map_visualizer_ =
      std::make_shared<SurfelMapVisualizer>(nh_, "map", "surfel_map");
  reflection_column_visualizer_ = std::make_shared<ReflectionColumnVisualizer>(
      nh_, "map", "reflection_column");
  tag_pose_array_visualizer_ =
      std::make_shared<PoseArrayVisualizer>(nh_, "map", "tag_map");
}

void LidarMapperRosVisualizer::PubExtrinsicTf() {
  static tf::TransformBroadcaster br;
  for (auto &st : sensor_tf_vec_) {
    st.stamp_ = ros::Time::now();
    br.sendTransform(st);
  }
}

void LidarMapperRosVisualizer::PubBaselinkTf(
    const Eigen::Matrix4d &_map_to_baselink, ros::Time _stamp) {
  static tf::TransformBroadcaster br;
  tf::Transform t = utils::TransdToTransform(_map_to_baselink);
  tf::StampedTransform st(t, _stamp, "map", "base_link");
  br.sendTransform(st);
}

}  // namespace multi_sensor_mapping
