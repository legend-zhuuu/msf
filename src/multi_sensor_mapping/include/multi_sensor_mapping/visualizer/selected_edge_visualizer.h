#ifndef MSM_SELECTED_EDGE_VISUALIZER_H
#define MSM_SELECTED_EDGE_VISUALIZER_H

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 选择的边显示器
 *
 */
class SelectedEdgeVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Selected Edge Visualizer object 构造函数
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  SelectedEdgeVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                         std::string _topic_name);

  /**
   * @brief 显示
   *
   * @param _graph
   * @param _stamp
   */
  void Display(const Eigen::Vector3d& _pos1, const Eigen::Vector3d& _pos2,
               ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  /// @brief 位姿图 marker
  visualization_msgs::MarkerArray pose_graph_markers_;
};
}  // namespace multi_sensor_mapping

#endif