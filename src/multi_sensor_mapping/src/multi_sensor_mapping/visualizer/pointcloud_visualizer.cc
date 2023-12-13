#include "multi_sensor_mapping/visualizer/pointcloud_visualizer.h"

namespace multi_sensor_mapping {

PointCloudVisualizer::PointCloudVisualizer(ros::NodeHandle _nh,
                                           std::string _frame_id,
                                           std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<sensor_msgs::PointCloud2>(_topic_name, 1);
}

void PointCloudVisualizer::Display(const CloudTypePtr& _cloud_raw,
                                   ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;

  pcl::toROSMsg(*_cloud_raw, pointcloud_msg_);
  pointcloud_msg_.header.frame_id = frame_id_;
  pointcloud_msg_.header.stamp = _stamp;
  pub_visual_.publish(pointcloud_msg_);
}

void PointCloudVisualizer::DisplayAgain(ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;
  pointcloud_msg_.header.stamp = _stamp;
  pub_visual_.publish(pointcloud_msg_);
}

}  // namespace multi_sensor_mapping