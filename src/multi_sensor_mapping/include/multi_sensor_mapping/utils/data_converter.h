#ifndef MSM_UTILS_DATA_CONVERTER_H
#define MSM_UTILS_DATA_CONVERTER_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_common.h"

namespace multi_sensor_mapping {

class ExtrinsicParams;

/**
 * @brief 数据转换器
 *
 */
class DataConverter {
 public:
  typedef std::shared_ptr<DataConverter> Ptr;

  /**
   * @brief DataConverter 构造函数
   *
   */
  DataConverter();

  /**
   * @brief DataConverter 构造函数
   * @param _min_range
   * @param _max_range
   */
  DataConverter(double _min_range, double _max_range);

  /**
   * @brief ConvertImuData 转换IMU数据
   * @param _imu_msg
   * @return
   */
  IMUData ConvertImuData(const sensor_msgs::Imu::ConstPtr& _imu_msg);

  /**
   * @brief ConvertImuData2 转换IMU数据
   *
   * @param _imu_msg
   * @return IMUData2
   */
  IMUData2 ConvertImuData2(const sensor_msgs::Imu::ConstPtr& _imu_msg);

  /**
   * @brief ConvertOdomData 轮速里程计数据转换
   * @param _odom_msg
   * @return
   */
  OdometryData ConvertOdomData(const nav_msgs::Odometry::ConstPtr& _odom_msg);

  /**
   * @brief GnssDataConvert GNSS数据转换器
   * @param _gnss_msg
   * @return
   */
  GNSSData ConvertGnssData(const sensor_msgs::NavSatFix::ConstPtr& _gnss_msg);

  /**
   * @brief RsCloudDataConvert 点云数据转换(RsLidar format)
   *
   * @param _cloud_msg
   * @return TimedCloudData
   */
  TimedCloudData ConvertRsCloudData(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief ConvertCloudData2 点云数据转换(RsLidar format)
   *
   * @param _cloud_msg
   * @return TimedCloudData2
   */
  TimedCloudData2 ConvertRsCloudData2(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief ConvertLegacyRsCloudData 点云数据转换(Rslidar format 老版本)
   *
   * @param _cloud_msg
   * @return TimedCloudData
   */
  TimedCloudData ConvertLegacyRsCloudData(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief ConvertLegacyRsCloudData 点云数据转换(Rslidar format 老版本)
   *
   * @param _cloud_msg
   * @return TimedCloudData
   */
  TimedCloudData2 ConvertLegacyRsCloudData2(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief VelCloudDataConvert 点云数据转换(Velodyne format)
   *
   * @param _cloud_msg

   * @return TimedCloudData
   */
  TimedCloudData ConvertVelCloudData(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief ConvertHesaiCloudData 点云数据转换(Hesai format)
   *
   * @param _cloud_msg
   * @return TimedCloudData
   */
  TimedCloudData ConvertHesaiCloudData(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief ConvertLivoxCloudData 点云数据转换(Livox format)
   *
   * @param _cloud_msg
   * @return TimedCloudData
   */
  TimedCloudData ConvertLivoxCloudData(
      const sensor_msgs::PointCloud2::ConstPtr _cloud_msg);

  /**
   * @brief PreprocessCloud 激光点云数据预处理(Rslidar)
   *
   * @param _cloud
   * @param output_cloud
   * @param output_timestamp
   * @return true
   * @return false
   */
  bool PreprocessCloud(RsRTPointCloudPtr _cloud,
                       VelRTPointCloudPtr& output_cloud,
                       double& output_timestamp);

  /**
   * @brief PreprocessCloud 数据点云预处理(Rslidar)
   *
   * @param _cloud
   * @param _output_cloud
   * @param _scan_start_time
   * @param _scan_end_time
   * @return true
   * @return false
   */
  bool PreprocessCloud(RsRTPointCloudPtr _cloud,
                       KRTPointCloudPtr& _output_cloud,
                       int64_t& _scan_start_time, int64_t& _scan_end_time);

  /**
   * @brief PreprocessCloud 激光点云数据预处理(Rslidar)
   *
   * @param _cloud
   * @param output_cloud
   * @param output_timestamp
   * @return true
   * @return false
   */
  bool PreprocessCloud(LegacyRsRTPointCloudPtr _cloud,
                       VelRTPointCloudPtr& output_cloud,
                       double& output_timestamp);

  /**
   * @brief PreprocessCloud 数据点云预处理(Rslidar)
   *
   * @param _cloud
   * @param _output_cloud
   * @param _scan_start_time
   * @param _scan_end_time
   * @return true
   * @return false
   */
  bool PreprocessCloud(LegacyRsRTPointCloudPtr _cloud,
                       KRTPointCloudPtr& _output_cloud,
                       int64_t& _scan_start_time, int64_t& _scan_end_time);

  /**
   * @brief PreprocessCloud 激光点云数据预处理(Velodyne)
   *
   * @param cloud
   * @param output_cloud
   * @param output_timestamp
   * @return true
   * @return false
   */
  bool PreprocessCloud(VelRTPointCloudPtr& cloud,
                       VelRTPointCloudPtr& output_cloud,
                       double& output_timestamp);

  /**
   * @brief PreprocessCloud 激光点云数据预处理(Hesai)
   *
   * @param _cloud
   * @param output_cloud
   * @param output_timestamp
   * @return true
   * @return false
   */
  bool PreprocessCloud(HsRTPointCloudPtr _cloud,
                       VelRTPointCloudPtr& output_cloud,
                       double& output_timestamp);

  /**
   * @brief PreprocessCloud 激光点云数据预处理(Livox)
   *
   * @param _cloud
   * @param _output_cloud
   * @param _output_timestamp
   * @return true
   * @return false
   */
  bool PreprocessCloud(LvRTPointCloudPtr _cloud,
                       VelRTPointCloudPtr& _output_cloud,
                       double& _output_timestamp);

 private:
  /// @brief 激光最近距离平方
  double lidar_min_range_square_;
  /// @brief 激光最远距离平方
  double lidar_max_range_square_;
};
}  // namespace multi_sensor_mapping

#endif