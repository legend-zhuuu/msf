/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef VELODYNE_TRANSFORM_H
#define VELODYNE_TRANSFORM_H

#include <dataset_adapter/VelodyneRawData.h>

namespace dataset_adapter {

/**
 * @brief The VelodyneTransform class
 * Adapted from velodyne
 */
class VelodyneTransform {
 public:
  typedef std::shared_ptr<VelodyneTransform> Ptr;

  /**
   * @brief VelodyneTransform 构造函数
   * @param lidar_model
   * @param lidar_frame
   * @param calibration_path
   * @param min_range
   * @param max_range
   * @param view_direction
   * @param view_width
   */
  VelodyneTransform(std::string lidar_model, std::string lidar_frame,
                    std::string calibration_path, double min_range,
                    double max_range, double view_direction, double view_width);

  /**
   * @brief ProcessScan 处理激光帧
   * @param scan_msg
   */
  void ProcessScan(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg,
                   sensor_msgs::PointCloud2::Ptr out_msg);

 private:
  boost::shared_ptr<VelodyneRawData> data_;

  std::string lidar_frame_;
};

}  // namespace dataset_adapter

#endif
