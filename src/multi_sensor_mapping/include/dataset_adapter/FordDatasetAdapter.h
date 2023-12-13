/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef FORD_DATASET_ADAPTER_H
#define FORD_DATASET_ADAPTER_H

#include <multi_sensor_localization/utils/UtilsEigen.h>
#include <multi_sensor_localization/utils/UtilsSensorData.h>
#include <Eigen/Eigen>

namespace dataset_adapter {

/**
 * @brief The FordDatasetAdapter class  Ford数据集适配器
 */
class FordDatasetAdapter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FordDatasetAdapter> Ptr;

  /**
   * @brief FordDatasetAdapter
   * @param cache_path
   */
  FordDatasetAdapter(std::string cache_path);

  /**
   * @brief SetSensorEnableFlag
   * @param use_imu
   * @param use_yellow_lidar
   * @param use_red_lidar
   * @param use_blue_lidar
   * @param use_green_lidar
   * @param use_gps
   */
  void SetSensorEnableFlag(bool use_imu, bool use_yellow_lidar,
                           bool use_red_lidar, bool use_blue_lidar,
                           bool use_green_lidar, bool use_gps);

  /**
   * @brief GenerateMap 生成地图
   * @param map_path
   */
  void GenerateMap(std::string map_path);

  /**
   * @brief ParseRawBag 解析原始数据包
   * @param bag_path
   */
  void ParseRawBag(std::string bag_path, std::string lidar_model,
                   std::string calibration_path, double min_range = 0.9,
                   double max_range = 100.0, double view_direction = 0,
                   double view_width = 2 * M_PI);

  /**
   * @brief WriteExtrinsicParams 写外参数据
   * @param path
   */
  void WriteExtrinsicParams(std::string path);

  /**
   * @brief RedVelodyneMergeMap 地图拼接
   * @param bag_path
   */
  void RedVelodyneMergeMap(std::string bag_path);

 private:
  /**
   * @brief GetFileNames 获取文件中的PCD文件
   * @param folder_path
   * @param pcd_paths
   */
  bool GetFileNames(std::string folder_path,
                    std::vector<std::string>& pcd_paths);

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
  /// @brief 保存数据路径
  std::string cache_path_;

  /// @brief 使用IMU数据标志位
  bool use_imu_flag_;
  /// @brief 使用激光雷达数据标志位（左1）
  bool use_yellow_lidar_flag_;
  /// @brief 使用激光雷达数据标志位(左2)
  bool use_red_lidar_flag_;
  /// @brief 使用激光雷达数据标志位(右2)
  bool use_blue_lidar_flag_;
  /// @brief 使用激光雷达数据标志位(右1)
  bool use_green_lidar_flag_;
  /// @brief 使用gps数据标志位
  bool use_gps_flag_;

  /// @brief 数据topic
  std::string imu_topic_;
  std::string yellow_velodyne_topic_;
  std::string red_velodyne_topic_;
  std::string blue_velodyne_topic_;
  std::string green_velodyne_topic_;
  std::string gps_topic_;

  /// @brief 真值姿态
  Eigen::aligned_vector<multi_sensor_localization::PoseData> red_vlp_gt_poses_;

  /// @brief red激光外参
  Eigen::Vector3f p_red_vlp_in_vehicle_;
  Eigen::Quaternionf q_red_vlp_in_vehicle_;
};

}  // namespace dataset_adapter

#endif
