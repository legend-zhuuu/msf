/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef KAIST_DATASET_ADAPTER_H
#define KAIST_DATASET_ADAPTER_H

#include <multi_sensor_localization/utils/UtilsEigen.h>
#include <multi_sensor_localization/utils/UtilsSensorData.h>
#include <Eigen/Eigen>

namespace dataset_adapter {

/**
 * @brief The KaistDatasetAdapter class KAIST数据集适配器
 */
class KaistDatasetAdapter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<KaistDatasetAdapter> Ptr;

  /**
   * @brief KaistDatasetAdapter 构造函数
   * @param path
   */
  KaistDatasetAdapter(std::string path);

  /**
   * @brief SetSensorEnableFlag 设置传感器使能
   * @param use_imu
   * @param use_left_lidar
   * @param use_right_lidar
   * @param use_sick
   * @param use_wheel_odom
   * @param use_stereo
   * @param use_gps
   */
  void SetSensorEnableFlag(bool use_imu, bool use_left_lidar,
                           bool use_right_lidar, bool use_sick,
                           bool use_wheel_odom, bool use_stereo, bool use_gps);

  /**
   * @brief WriteDataToRosbag 写入数据包
   */
  void WriteDataToRosbag();

 private:
  /**
   * @brief WriteVelodyneDataToBag 写velodyne数据到rosbag
   * @param path
   * @param bag_ptr
   */
  void WriteVelodyneDataToBag(std::string path,
                              std::shared_ptr<rosbag::Bag> bag_ptr);

  /**
   * @brief WriteImuDataToBag 写IMU数据到rosbag
   * @param path
   * @param bag_ptr
   */
  void WriteImuDataToBag(std::string path,
                         std::shared_ptr<rosbag::Bag> bag_ptr);

  /**
   * @brief WriteWheelOdomDataToBag 写轮速计数据到rosbag
   * @param path
   * @param bag_ptr
   */
  void WriteWheelOdomDataToBag(std::string path,
                               std::shared_ptr<rosbag::Bag> bag_ptr);

  /**
   * @brief WriteSickDataToBag 写SICK雷达数据到rosbag
   * @param path
   * @param bag_ptr
   */
  void WriteSickDataToBag(std::string path,
                          std::shared_ptr<rosbag::Bag> bag_ptr);

  /**
   * @brief WriteExtrinsicParams 读取外参并写入
   * @param path
   */
  void WriteExtrinsicParams(std::string path);

  /**
   * @brief ComputeVLP16Ring 通过xyz计算ring
   * @param raw_point
   * @return
   */
  int ComputeVLP16Ring(const VelRTPoint& raw_point);

  /**
   * @brief TransformGroundTruthToTumFormat 真值转换为TUM格式
   * @param path
   */
  void TransformGroundTruthToTumFormat(std::string path);

  /**
   * @brief RightVelodyneMergeMap 有激光雷达生成地图
   * @param bag_path
   */
  void RightVelodyneMergeMap(std::string bag_path);

  /**
   * @brief SlerpPose 姿态插值
   * @param pose0
   * @param pose1
   * @param pose_out
   */
  void SlerpPose(const multi_sensor_localization::PoseData& pose0,
                 const multi_sensor_localization::PoseData& pose1,
                 multi_sensor_localization::PoseData& pose_out);

  /**
   * @brief UndistortedScan 姿态矫正
   * @param odom_in_Lk_vec
   * @param scan_time
   * @param raw_cloud
   * @param output_cloud
   */
  void UndistortedScan(const Eigen::aligned_vector<
                           multi_sensor_localization::PoseData>& odom_in_Lk_vec,
                       double scan_time, VelRTPointCloudPtr raw_cloud,
                       CloudTypePtr output_cloud);

 private:
  /// @brief 数据路径
  std::string data_path_;

  /// @brief 使用IMU数据标志位
  bool use_imu_flag_;
  /// @brief 使用左激光雷达数据标志位
  bool use_left_lidar_flag_;
  /// @brief 使用右激光雷达数据标志位
  bool use_right_lidar_flag_;
  /// @brief 使用sick激光雷达数据标志位
  bool use_sick_flag_;
  /// @brief 使用轮速计标志位
  bool use_wheel_odom_flag_;
  /// @brief 使用双目数据标志位
  bool use_stereo_flag_;
  /// @brief 使用gps数据标志位
  bool use_gps_flag_;

  /// @brief 数据topic
  std::string imu_topic_;
  std::string left_velodyne_topic_;
  std::string right_velodyne_topic_;
  std::string front_sick_topic_;
  std::string back_sick_topic_;
  std::string wheel_odom_topic_;
  std::string left_stereo_topic_;
  std::string right_stereo_topic_;
  std::string gps_topic_;

  /// @brief right_vlp_gt_poses_ 右激光的真值姿态
  Eigen::aligned_vector<multi_sensor_localization::PoseData>
      right_vlp_gt_poses_;

  /// @brief 右激光外参
  Eigen::Vector3f p_right_vlp_in_vehicle_;
  Eigen::Quaternionf q_right_vlp_in_vehicle_;
};

}  // namespace dataset_adapter

#endif
