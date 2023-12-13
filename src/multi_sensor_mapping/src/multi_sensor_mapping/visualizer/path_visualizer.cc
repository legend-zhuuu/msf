#include "multi_sensor_mapping/visualizer/path_visualizer.h"

namespace multi_sensor_mapping {

PosePathVisualizer::PosePathVisualizer(ros::NodeHandle _nh,
                                       std::string _frame_id,
                                       std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<nav_msgs::Path>(_topic_name, 1);
  pose_path_.header.frame_id = frame_id_;
}

void PosePathVisualizer::Display(const Eigen::Matrix4d& _pose,
                                 ros::Time _stamp) {
  geometry_msgs::PoseStamped path_point;
  path_point.pose.position.x = _pose(0, 3);
  path_point.pose.position.y = _pose(1, 3);
  path_point.pose.position.z = _pose(2, 3);
  Eigen::Quaterniond q(_pose.block<3, 3>(0, 0));
  path_point.pose.orientation.w = q.w();
  path_point.pose.orientation.x = q.x();
  path_point.pose.orientation.y = q.y();
  path_point.pose.orientation.z = q.z();

  pose_path_.poses.push_back(path_point);

  if (pub_visual_.getNumSubscribers() == 0) return;
  pose_path_.header.stamp = _stamp;
  pub_visual_.publish(pose_path_);
}

void PosePathVisualizer::DisplayAgain(ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;
  pose_path_.header.stamp = _stamp;
  pub_visual_.publish(pose_path_);
}

void PosePathVisualizer::Reset() { pose_path_.poses.clear(); }

}  // namespace multi_sensor_mapping