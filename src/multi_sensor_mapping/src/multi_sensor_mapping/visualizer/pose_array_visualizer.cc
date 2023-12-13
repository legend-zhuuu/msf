#include "multi_sensor_mapping/visualizer/pose_array_visualizer.h"

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

PoseArrayVisualizer::PoseArrayVisualizer(ros::NodeHandle _nh,
                                         std::string _frame_id,
                                         std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<geometry_msgs::PoseArray>(topic_name_, 1);
  pose_array_msg_.header.frame_id = frame_id_;
}

void PoseArrayVisualizer::Display(const std::vector<PoseData>& _pose_vec,
                                  ros::Time _stamp) {
  pose_array_msg_.poses.clear();
  for (auto pose : _pose_vec) {
    geometry_msgs::Pose new_pose;
    new_pose.position.x = pose.position(0);
    new_pose.position.y = pose.position(1);
    new_pose.position.z = pose.position(2);

    new_pose.orientation.w = pose.orientation.w();
    new_pose.orientation.x = pose.orientation.x();
    new_pose.orientation.y = pose.orientation.y();
    new_pose.orientation.z = pose.orientation.z();
    pose_array_msg_.poses.push_back(new_pose);
  }

  pose_array_msg_.header.stamp = _stamp;

  pub_visual_.publish(pose_array_msg_);
}

void PoseArrayVisualizer::DisplayAgain(ros::Time _stamp) {
  pose_array_msg_.header.stamp = _stamp;
  pub_visual_.publish(pose_array_msg_);
}

}  // namespace multi_sensor_mapping