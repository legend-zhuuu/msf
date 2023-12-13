#include "multi_sensor_mapping/visualizer/visualizer_base.h"

#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

std_msgs::ColorRGBA GetColor(int _color_type) {
  std_msgs::ColorRGBA color;
  switch (_color_type) {
    case VISUALIZER_COLOR_RED:
      color.r = 1.0f;
      color.g = 0.0f;
      color.b = 0.0f;
      break;
    case VISUALIZER_COLOR_YELLOW:
      color.r = 1.0f;
      color.g = 1.0f;
      color.b = 0.0f;
      break;
    case VISUALIZER_COLOR_BLUE:
      color.r = 0.0f;
      color.g = 0.0f;
      color.b = 1.0f;
      break;
    case VISUALIZER_COLOR_GREEN:
      color.r = 0.0f;
      color.g = 1.0f;
      color.b = 0.0f;
      break;
    case VISUALIZER_COLOR_WHITE:
      color.r = 1.0f;
      color.g = 1.0f;
      color.b = 1.0f;
      break;
    case VISUALIZER_COLOR_PINK:
      color.r = 1.0f;
      color.g = 0.75f;
      color.b = 0.8f;
      break;
    case VISUALIZER_COLOR_CYAN:
      color.r = 0.0f;
      color.g = 1.0f;
      color.b = 1.0f;
      break;
    default:
      break;
  }
  return color;
}

std_msgs::ColorRGBA GetColor(int _color_type, double _transparency) {
  std_msgs::ColorRGBA color = GetColor(_color_type);
  color.a = _transparency;
  return color;
}

ColorPalette::ColorPalette(const std::vector<VisualizerColor> &_colors) {
  int color_size = _colors.size();
  if (color_size < 2) {
    AWARN_F("[ColorPalette] Please add multiple colors !");
  } else {
    seg_size_inv_ = 1.0 / (double)(color_size - 1);

    for (int j = 0; j < color_size; j++) {
      colors_.push_back(GetColor(_colors[j]));
      seg_.push_back(j * seg_size_inv_);
    }
  }
}

std_msgs::ColorRGBA ColorPalette::GetGradientColor(double _pos) {
  std_msgs::ColorRGBA rgb_msg;

  int idx = (int)colors_.size() - 1;
  for (; idx >= 0; idx--) {
    if (_pos > seg_[idx]) {
      break;
    }
  }

  if (idx < 0) idx = 0;

  double color_cofee = (_pos - seg_[idx]) / seg_size_inv_;
  rgb_msg.r =
      colors_[idx].r + color_cofee * (colors_[idx + 1].r - colors_[idx].r);
  rgb_msg.g =
      colors_[idx].g + color_cofee * (colors_[idx + 1].g - colors_[idx].g);
  rgb_msg.b =
      colors_[idx].b + color_cofee * (colors_[idx + 1].b - colors_[idx].b);
  return rgb_msg;
}

VisualizerBase::VisualizerBase(ros::NodeHandle _node_handle,
                               std::string _frame_id, std::string _topic_name)
    : nh_(_node_handle), frame_id_(_frame_id), topic_name_(_topic_name) {}

}  // namespace multi_sensor_mapping