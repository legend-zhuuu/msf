/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Lv Jiajun, Hu kewei, Wu Hangyu, et al. (see THANKS for the full
 author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef LIDAR_TIMESTAMP_RECOVERY_H
#define LIDAR_TIMESTAMP_RECOVERY_H

#include <multi_sensor_localization/utils/UtilsSensorData.h>

namespace dataset_adapter {

/**
 * @brief The LidarTimestampRecovery class 激光雷达时间戳恢复(VLP16)
 * Adapted from CLIC
 */
class LidarTimestampRecovery {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief LidarTimestampRecovery 构造函数
   */
  LidarTimestampRecovery();

  /**
   * @brief SetInputCloud  输入点云
   * @param raw_cloud
   */
  void SetInputCloud(const VelRTPointCloudPtr& raw_cloud);

  /**
   * @brief RecoverCloud 恢复时间戳
   * @param index
   * @param raw_cloud
   */
  void RecoverTimestamp(int index, VelRTPointCloudPtr& raw_cloud);

  /**
   * @brief Reset
   */
  void Reset();

  /**
   * @brief CorrectStartAngleOfScan 矫正激光初始角度
   */
  void CorrectStartAngleOfScan();

 private:
  /**
   * @brief RecoverPointCloudTimestmap
   * @param raw_cloud
   * @param start_angle
   */
  void RecoverPointCloudTimestmap(VelRTPointCloudPtr raw_cloud,
                                  double start_angle = -1000);

  /**
   * @brief GetAngleOfScan
   * @param raw_cloud
   * @param start_angle
   * @param rotation_travelled
   */
  void GetAngleOfScan(const VelRTPointCloudPtr& raw_cloud, double& start_angle,
                      double& rotation_travelled);

  /**
   * @brief ClockwiseAngle
   * @param bef_angle
   * @param after_angle
   * @return
   */
  double ClockwiseAngle(double bef_angle, double after_angle);

  /**
   * @brief RotationTravelledClockwise
   * @param now_angle
   * @param reset_cnt
   * @return
   */
  double RotationTravelledClockwise(double now_angle, bool reset_cnt = false);

  /**
   * @brief LineFitting 拟合线
   * @param point
   * @param line_k_b
   */
  void LineFitting(const std::vector<double>& point, Eigen::Vector2d& line_k_b);

 private:
  /// @brief 起始点角度
  std::vector<double> start_point_angle_vec_;
  /// @brief 旋转过的角度
  std::vector<double> rotation_travelled_angle_vec_;
  /// @brief 矫正后的起始角度
  std::vector<double> correct_start_angle_vec_;
  /// @brief 一帧激光雷达转过的角度(°)
  double one_scan_angle_;

  double angle2time_;
};

}  // namespace dataset_adapter

#endif
