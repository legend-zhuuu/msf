#include "multi_sensor_mapping/visualizer/reflection_column_visualizer.h"

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

ReflectionColumnVisualizer::ReflectionColumnVisualizer(ros::NodeHandle _nh,
                                                       std::string _frame_id,
                                                       std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<visualization_msgs::MarkerArray>(_topic_name, 1);
}

void ReflectionColumnVisualizer::Display(
    const std::vector<ReflectionColumn>& _reflection_columns,
    ros::Time _stamp) {
  reflection_column_markers_.markers.clear();

  for (size_t i = 0; i < _reflection_columns.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = _stamp;
    marker.ns = "map";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = _reflection_columns[i].diamater / 2;
    marker.scale.y = _reflection_columns[i].diamater / 2;
    marker.scale.z = _reflection_columns[i].length;
    marker.color.g = 1.0;
    marker.color.a = 0.8;
    marker.pose.position.x = _reflection_columns[i].position.x();
    marker.pose.position.y = _reflection_columns[i].position.y();
    marker.pose.position.z = _reflection_columns[i].position.z();

    reflection_column_markers_.markers.push_back(marker);
  }

  pub_visual_.publish(reflection_column_markers_);
}

void ReflectionColumnVisualizer::DisplayAgain(ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;

  pub_visual_.publish(reflection_column_markers_);
}

}  // namespace multi_sensor_mapping