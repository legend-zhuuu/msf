#include "multi_sensor_mapping/visualizer/odom_visualizer.h"

namespace multi_sensor_mapping {

PoseOdomVisualizer::PoseOdomVisualizer(ros::NodeHandle _nh,
                                       std::string _frame_id,
                                       std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<nav_msgs::Odometry>(_topic_name, 1);
  odom_msg_.header.frame_id = frame_id_;
}

void PoseOdomVisualizer::Display(const Eigen::Matrix4d& _pose,
                                 ros::Time _stamp) {
  Eigen::Quaterniond pose_q(_pose.block<3, 3>(0, 0));

  odom_msg_.pose.pose.position.x = _pose(0, 3);
  odom_msg_.pose.pose.position.y = _pose(1, 3);
  odom_msg_.pose.pose.position.z = _pose(2, 3);
  odom_msg_.pose.pose.orientation.w = pose_q.w();
  odom_msg_.pose.pose.orientation.x = pose_q.x();
  odom_msg_.pose.pose.orientation.y = pose_q.y();
  odom_msg_.pose.pose.orientation.z = pose_q.z();

  odom_msg_.header.stamp = _stamp;

  pub_visual_.publish(odom_msg_);
}

}  // namespace multi_sensor_mapping