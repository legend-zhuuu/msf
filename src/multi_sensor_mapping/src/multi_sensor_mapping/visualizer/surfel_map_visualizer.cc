#include "multi_sensor_mapping/visualizer/surfel_map_visualizer.h"

#include "multi_sensor_mapping/map/surfel_octo_map.h"

namespace multi_sensor_mapping {

SurfelMapVisualizer::SurfelMapVisualizer(ros::NodeHandle _nh,
                                         std::string _frame_id,
                                         std::string _topic_name)
    : VisualizerBase(_nh, _frame_id, _topic_name) {
  pub_visual_ = nh_.advertise<visualization_msgs::MarkerArray>(_topic_name, 1);
}

void SurfelMapVisualizer::Display(const std::vector<SurfelUnit>& _surfel_units,
                                  ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;

  surfel_map_markers_.markers.clear();

  // 初始化调色盘
  std::vector<VisualizerColor> colors;
  colors.push_back(VISUALIZER_COLOR_GREEN);
  colors.push_back(VISUALIZER_COLOR_CYAN);
  colors.push_back(VISUALIZER_COLOR_BLUE);
  ColorPalette palette(colors);

  size_t surfel_num = _surfel_units.size();
  double depth_num = 5;
  for (size_t i = 0; i < surfel_num; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = _stamp;
    marker.ns = "surfel_map";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = _surfel_units[i].mean(0);
    marker.pose.position.y = _surfel_units[i].mean(1);
    marker.pose.position.z = _surfel_units[i].mean(2);
    marker.pose.orientation.x = _surfel_units[i].direction.x();
    marker.pose.orientation.y = _surfel_units[i].direction.y();
    marker.pose.orientation.z = _surfel_units[i].direction.z();
    marker.pose.orientation.w = _surfel_units[i].direction.w();
    marker.scale.x = 0.01;
    marker.scale.y = _surfel_units[i].leaf_resolution / 2;
    marker.scale.z = _surfel_units[i].leaf_resolution / 2;

    // marker.scale.x = _surfel_units[i].eigen_value(0) /
    //                  _surfel_units[i].eigen_value.sum() *
    //                  _surfel_units[i].leaf_resolution;
    // marker.scale.y = _surfel_units[i].eigen_value(1) /
    //                  _surfel_units[i].eigen_value.sum() *
    //                  _surfel_units[i].leaf_resolution;
    // marker.scale.z = _surfel_units[i].eigen_value(2) /
    //                  _surfel_units[i].eigen_value.sum() *
    //                  _surfel_units[i].leaf_resolution;

    double color_scale = (double)_surfel_units[i].depth_level / depth_num;
    auto rgba = palette.GetGradientColor(color_scale);
    rgba.a = 0.8;
    marker.color = rgba;
    surfel_map_markers_.markers.push_back(marker);
  }

  pub_visual_.publish(surfel_map_markers_);
}

void SurfelMapVisualizer::DisplayAgain(ros::Time _stamp) {
  if (pub_visual_.getNumSubscribers() == 0) return;

  pub_visual_.publish(surfel_map_markers_);
}

}  // namespace multi_sensor_mapping