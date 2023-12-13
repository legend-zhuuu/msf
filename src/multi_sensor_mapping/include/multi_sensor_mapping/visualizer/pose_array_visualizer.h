#ifndef MSM_POSE_ARRAY_VISUALIZER_H
#define MSM_POSE_ARRAY_VISUALIZER_H

#include <geometry_msgs/PoseArray.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

struct PoseData;

/**
 * @brief 位姿可视化器
 *
 */
class PoseArrayVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Pose Visualizer object
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  PoseArrayVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                      std::string _topic_name);

  /**
   * @brief Display 可视化位姿列表
   * @param _pose_vec
   */
  void Display(const std::vector<PoseData>& _pose_vec,
               ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  /// @brief 位置消息
  geometry_msgs::PoseArray pose_array_msg_;
};

}  // namespace multi_sensor_mapping

#endif