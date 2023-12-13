#include "multi_sensor_mapping/visualizer/pose_graph_visualizer.h"

#include "multi_sensor_mapping/backend/pose_graph_information.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"

namespace multi_sensor_mapping {

Triangle3D::Triangle3D() {
  vertex1_(0) = 0;
  vertex1_(1) = 0.5 * 0.1;
  vertex1_(2) = 0;

  vertex2_(0) = 0;
  vertex2_(1) = -0.5 * 0.1;
  vertex2_(2) = 0;

  vertex3_(0) = 0.15;
  vertex3_(1) = 0;
  vertex3_(2) = 0;
}

Triangle3D::Triangle3D(const double _length, const double _width) {
  vertex1_(0) = 0;
  vertex1_(1) = 0.5 * _width;
  vertex1_(2) = 0;

  vertex2_(0) = 0;
  vertex2_(1) = -0.5 * _width;
  vertex2_(2) = 0;

  vertex3_(0) = _length;
  vertex3_(1) = 0;
  vertex3_(2) = 0;
}

std::vector<Eigen::Vector3d> Triangle3D::Transform(
    const Eigen::Vector3d& _translation, const Eigen::Quaterniond& _rotation) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
  mat.block<3, 1>(0, 3) = _translation;
  mat.block<3, 3>(0, 0) = _rotation.toRotationMatrix();
  return Transform(mat);
}

std::vector<Eigen::Vector3d> Triangle3D::Transform(
    const Eigen::Matrix4d& _mat) {
  std::vector<Eigen::Vector3d> transformed_vertexs;

  transformed_vertexs.push_back(utils::TransformPoint(vertex1_, _mat));
  transformed_vertexs.push_back(utils::TransformPoint(vertex2_, _mat));
  transformed_vertexs.push_back(utils::TransformPoint(vertex3_, _mat));

  return transformed_vertexs;
}

PoseGraphVisualizer::PoseGraphVisualizer(ros::NodeHandle _nh,
                                         std::string _frame_id,
                                         std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<visualization_msgs::MarkerArray>(_topic_name, 1);
}

void PoseGraphVisualizer::Display(const PoseGraph& _graph, ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;

  pose_graph_markers_.markers.clear();

  Triangle3D triangle(0.5, 0.3);

  // 初始化调色盘
  std::vector<VisualizerColor> colors;
  colors.push_back(VISUALIZER_COLOR_RED);
  colors.push_back(VISUALIZER_COLOR_GREEN);
  ColorPalette palette(colors);

  // 添加顶点
  visualization_msgs::Marker marker_vertex;
  geometry_msgs::Point p1, p2, p3;
  marker_vertex.action = visualization_msgs::Marker::MODIFY;
  marker_vertex.header.frame_id = frame_id_;
  marker_vertex.ns = "vertex";
  marker_vertex.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker_vertex.scale.x = 1;
  marker_vertex.scale.y = 1;
  marker_vertex.scale.z = 1;
  marker_vertex.pose.orientation.w = 1;
  marker_vertex.id = 0;

  int vertex_size = (int)_graph.vertices.size();

  marker_vertex.points.resize(3 * vertex_size);
  marker_vertex.colors.resize(3 * vertex_size);

  for (int i = 0; i < vertex_size; i++) {
    auto graph_vertex = _graph.vertices[i];
    std::vector<Eigen::Vector3d> tri_vertex =
        triangle.Transform(graph_vertex.position, graph_vertex.orientation);

    p1.x = tri_vertex[0](0);
    p1.y = tri_vertex[0](1);
    p1.z = tri_vertex[0](2);
    p2.x = tri_vertex[1](0);
    p2.y = tri_vertex[1](1);
    p2.z = tri_vertex[1](2);
    p3.x = tri_vertex[2](0);
    p3.y = tri_vertex[2](1);
    p3.z = tri_vertex[2](2);
    marker_vertex.points[i * 3] = p1;
    marker_vertex.points[i * 3 + 1] = p2;
    marker_vertex.points[i * 3 + 2] = p3;

    auto rgba = palette.GetGradientColor((double)i / (double)vertex_size);
    rgba.a = 1.0f;

    marker_vertex.colors[i * 3] = rgba;
    marker_vertex.colors[i * 3 + 1] = rgba;
    marker_vertex.colors[i * 3 + 2] = rgba;
  }
  pose_graph_markers_.markers.push_back(marker_vertex);

  // 添加里程计边
  visualization_msgs::Marker marker_edge;
  marker_edge.action = visualization_msgs::Marker::MODIFY;
  marker_edge.header.frame_id = frame_id_;
  marker_edge.ns = "edge";
  marker_edge.type = visualization_msgs::Marker::LINE_LIST;
  marker_edge.scale.x = 0.02;
  marker_edge.pose.orientation.w = 1;
  marker_edge.id = 1;
  marker_edge.color = GetColor(VISUALIZER_COLOR_YELLOW, 0.5);

  for (int i = 1; i < vertex_size; i++) {
    p1.x = _graph.vertices[i - 1].position(0);
    p1.y = _graph.vertices[i - 1].position(1);
    p1.z = _graph.vertices[i - 1].position(2);

    p2.x = _graph.vertices[i].position(0);
    p2.y = _graph.vertices[i].position(1);
    p2.z = _graph.vertices[i].position(2);

    marker_edge.points.push_back(p1);
    marker_edge.points.push_back(p2);
  }
  pose_graph_markers_.markers.push_back(marker_edge);

  // 添加闭环边
  visualization_msgs::Marker loop_closure_edge;
  loop_closure_edge.action = visualization_msgs::Marker::MODIFY;
  loop_closure_edge.header.frame_id = frame_id_;
  loop_closure_edge.ns = "loop_closure";
  loop_closure_edge.type = visualization_msgs::Marker::LINE_LIST;
  loop_closure_edge.scale.x = 0.1;
  loop_closure_edge.pose.orientation.w = 1;
  loop_closure_edge.id = 3;
  loop_closure_edge.color = GetColor(VISUALIZER_COLOR_CYAN, 1.0);

  int loop_closure_size = _graph.loop_closure_edegs_.size();
  for (int i = 0; i < loop_closure_size; i++) {
    int front_id = _graph.loop_closure_edegs_[i].front_id;
    int back_id = _graph.loop_closure_edegs_[i].back_id;
    p1.x = _graph.vertices[front_id].position(0);
    p1.y = _graph.vertices[front_id].position(1);
    p1.z = _graph.vertices[front_id].position(2);

    p2.x = _graph.vertices[back_id].position(0);
    p2.y = _graph.vertices[back_id].position(1);
    p2.z = _graph.vertices[back_id].position(2);

    loop_closure_edge.points.push_back(p1);
    loop_closure_edge.points.push_back(p2);
  }
  if (loop_closure_size > 0) {
    pose_graph_markers_.markers.push_back(loop_closure_edge);
  }

  pub_visual_.publish(pose_graph_markers_);
}

void PoseGraphVisualizer::DisplayAgain(ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;
  for (size_t i = 0; i < pose_graph_markers_.markers.size(); i++) {
    pose_graph_markers_.markers[i].header.stamp = _stamp;
  }
  pub_visual_.publish(pose_graph_markers_);
}

}  // namespace multi_sensor_mapping
