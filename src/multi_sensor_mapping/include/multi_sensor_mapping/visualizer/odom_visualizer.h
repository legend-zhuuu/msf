#ifndef MSM_ODOM_VISUALIZER_H
#define MSM_ODOM_VISUALIZER_H

#include <nav_msgs/Odometry.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 里程计可视化
 *
 */
class PoseOdomVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Pose Odom Visualizer object 构造函数
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  PoseOdomVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                     std::string _topic_name);

  /**
   * @brief Display 可视化轨迹
   * @param _pose
   * @param _stamp
   */
  void Display(const Eigen::Matrix4d& _pose,
               ros::Time _stamp = ros::Time::now());

 private:
  nav_msgs::Odometry odom_msg_;
};
}  // namespace multi_sensor_mapping
#endif
