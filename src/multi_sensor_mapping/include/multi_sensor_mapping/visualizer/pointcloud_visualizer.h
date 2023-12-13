#ifndef MSM_POINTCLOUD_VISUALIZER_H
#define MSM_POINTCLOUD_VISUALIZER_H

#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "multi_sensor_mapping/visualizer/visualizer_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 点云显示器
 *
 */
class PointCloudVisualizer : public VisualizerBase {
 public:
  /**
   * @brief Construct a new Point Cloud Visualizer object
   *
   * @param _nh
   * @param _frame_id
   * @param _topic_name
   */
  PointCloudVisualizer(ros::NodeHandle _nh, std::string _frame_id,
                       std::string _topic_name);

  /**
   * @brief 显示
   *
   * @param _cloud_raw
   * @param _stamp
   */
  void Display(const CloudTypePtr& _cloud_raw,
               ros::Time _stamp = ros::Time::now());

  /**
   * @brief 重复显示
   *
   * @param _stamp
   */
  void DisplayAgain(ros::Time _stamp = ros::Time::now());

 private:
  sensor_msgs::PointCloud2 pointcloud_msg_;
};

}  // namespace multi_sensor_mapping

#endif