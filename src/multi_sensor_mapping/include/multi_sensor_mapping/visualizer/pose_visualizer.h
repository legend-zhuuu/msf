#ifndef MSM_POSE_VISUALIZER_H
#define MSM_POSE_VISUALIZER_H

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 位姿可视化器
 *
 */
class PoseVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Pose Visualizer object
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  PoseVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                 std::string _topic_name);

  /**
   * @brief Display 可视化位姿列表
   * @param _pose_vec
   */
  void Display(const Eigen::Matrix4d& _pose,
               ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  /// @brief 位置消息
  geometry_msgs::PoseStamped pose_msg_;
};

}  // namespace multi_sensor_mapping

#endif