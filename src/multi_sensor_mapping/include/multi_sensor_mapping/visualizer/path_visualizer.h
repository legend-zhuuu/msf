#ifndef MSM_PATH_VISUALIZER_H
#define MSM_PATH_VISUALIZER_H

#include <nav_msgs/Path.h>

#include <Eigen/Eigen>

#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 位姿可视化器
 *
 */
class PosePathVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Pose Path Visualizer object 构造函数
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  PosePathVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                     std::string _topic_name);

  /**
   * @brief 显示
   *
   * @param _pose
   * @param _stamp
   */
  void Display(const Eigen::Matrix4d& _pose,
               ros::Time _stamp = ros::Time::now());
  /**
   * @brief DisplayAgain 重复发布一次可视化消息
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重置
   *
   */
  void Reset();

 private:
  nav_msgs::Path pose_path_;
};
}  // namespace multi_sensor_mapping

#endif
