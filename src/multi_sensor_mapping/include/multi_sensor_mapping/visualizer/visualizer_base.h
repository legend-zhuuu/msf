#ifndef MSM_VISUALIZER_BASE_H
#define MSM_VISUALIZER_BASE_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>

namespace multi_sensor_mapping {

enum VisualizerColor {
  VISUALIZER_COLOR_RED = 0,
  VISUALIZER_COLOR_BLUE,
  VISUALIZER_COLOR_GREEN,
  VISUALIZER_COLOR_YELLOW,
  VISUALIZER_COLOR_WHITE,
  VISUALIZER_COLOR_PINK,
  VISUALIZER_COLOR_CYAN
};

/**
 * @brief Get the Color object 获取颜色
 *
 * @param _color_type
 * @return std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA GetColor(int _color_type);

/**
 * @brief Get the Color object 获取颜色
 *
 * @param _color_type
 * @param _transparency
 * @return std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA GetColor(int _color_type, double _transparency);

/**
 * @brief 调色器
 *
 */
class ColorPalette {
 public:
  /**
   * @brief Construct a new Color Palette object
   *
   * @param _colors
   */
  ColorPalette(const std::vector<VisualizerColor>& _colors);

  /**
   * @brief Get the GetGradientColor object
   *
   * @param _pos
   * @return std_msgs::ColorRGBA
   */
  std_msgs::ColorRGBA GetGradientColor(double _pos);

 private:
  std::vector<std_msgs::ColorRGBA> colors_;

  std::vector<double> seg_;

  double seg_size_inv_;
};

/**
 * @brief 可视化器基类
 *
 */
class VisualizerBase {
 public:
  /**
   * @brief Construct a new Visualizer Base object
   *
   */
  VisualizerBase(ros::NodeHandle _node_handle, std::string _frame_id,
                 std::string _topic_name);

 protected:
  /// @brief ros节点
  ros::NodeHandle nh_;
  /// @brief 发布器
  ros::Publisher pub_visual_;
  /// @brief 坐标系
  std::string frame_id_;
  /// @brief topic名字
  std::string topic_name_;
};

}  // namespace multi_sensor_mapping

#endif