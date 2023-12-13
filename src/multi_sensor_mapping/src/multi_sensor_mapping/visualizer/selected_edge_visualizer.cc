#include "multi_sensor_mapping/visualizer/selected_edge_visualizer.h"

namespace multi_sensor_mapping {

SelectedEdgeVisualizer::SelectedEdgeVisualizer(ros::NodeHandle _nh,
                                               std::string _frame_id,
                                               std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<visualization_msgs::MarkerArray>(_topic_name, 1);
}

void SelectedEdgeVisualizer::Display(const Eigen::Vector3d& _pos1,
                                     const Eigen::Vector3d& _pos2,
                                     ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;

  pose_graph_markers_.markers.clear();

  geometry_msgs::Point p1, p2;
  p1.x = _pos1(0);
  p1.y = _pos1(1);
  p1.z = _pos1(2);
  p2.x = _pos2(0);
  p2.y = _pos2(1);
  p2.z = _pos2(2);

  // 添加顶点
  visualization_msgs::Marker marker_vertex;
  marker_vertex.action = visualization_msgs::Marker::MODIFY;
  marker_vertex.header.frame_id = frame_id_;
  marker_vertex.ns = "selected_vertex";
  marker_vertex.type = visualization_msgs::Marker::SPHERE_LIST;
  marker_vertex.scale.x = 0.3;
  marker_vertex.scale.y = 0.3;
  marker_vertex.scale.z = 0.3;
  marker_vertex.pose.orientation.w = 1;
  marker_vertex.id = 0;
  marker_vertex.color = GetColor(VISUALIZER_COLOR_GREEN, 0.8);

  marker_vertex.points.push_back(p1);
  marker_vertex.points.push_back(p2);

  visualization_msgs::Marker marker_edge;
  marker_edge.action = visualization_msgs::Marker::MODIFY;
  marker_edge.header.frame_id = frame_id_;
  marker_edge.ns = "selected_edge";
  marker_edge.type = visualization_msgs::Marker::ARROW;
  marker_edge.scale.x = 0.3;
  marker_edge.scale.y = 0.5;
  marker_edge.pose.orientation.w = 1;
  marker_edge.id = 1;
  marker_edge.color = GetColor(VISUALIZER_COLOR_PINK, 1.0);
  marker_edge.points.push_back(p1);
  marker_edge.points.push_back(p2);

  pose_graph_markers_.markers.push_back(marker_vertex);
  pose_graph_markers_.markers.push_back(marker_edge);

  pub_visual_.publish(pose_graph_markers_);
}

void SelectedEdgeVisualizer::DisplayAgain(ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;
  for (size_t i = 0; i < pose_graph_markers_.markers.size(); i++) {
    pose_graph_markers_.markers[i].header.stamp = _stamp;
  }
  pub_visual_.publish(pose_graph_markers_);
}

}  // namespace multi_sensor_mapping