#include "multi_sensor_mapping/visualizer/pose_visualizer.h"

namespace multi_sensor_mapping {

PoseVisualizer::PoseVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                               std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_name_, 1);
  pose_msg_.header.frame_id = frame_id_;
}

void PoseVisualizer::Display(const Eigen::Matrix4d& _pose, ros::Time _stamp) {
  geometry_msgs::Pose new_pose;
  new_pose.position.x = _pose(0, 3);
  new_pose.position.y = _pose(1, 3);
  new_pose.position.z = _pose(2, 3);
  Eigen::Quaterniond q(_pose.block<3, 3>(0, 0));
  new_pose.orientation.w = q.w();
  new_pose.orientation.x = q.x();
  new_pose.orientation.y = q.y();
  new_pose.orientation.z = q.z();

  pose_msg_.header.stamp = _stamp;
  pose_msg_.pose = new_pose;

  pub_visual_.publish(pose_msg_);
}

void PoseVisualizer::DisplayAgain(ros::Time _stamp) {
  pose_msg_.header.stamp = _stamp;
  pub_visual_.publish(pose_msg_);
}

}  // namespace multi_sensor_mapping