#ifndef MSM_DATA_MANAGER_H
#define MSM_DATA_MANAGER_H

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class DataConverter;
class SensorParams;

/**
 * @brief 数据管理器(读取轻量级数据)
 *
 */
class DataManager {
 public:
  typedef std::shared_ptr<DataManager> Ptr;

  /**
   * @brief Construct a new Data Manager object 构造函数
   *
   * @param _data_converter
   */
  DataManager(const std::shared_ptr<DataConverter>& _data_converter);

  /**
   * @brief 从Rosbag中读取数据
   *
   * @param _bag_path
   * @param _sensor_params
   * @param _start_time
   * @param _bag_durr
   * @return true
   * @return false
   */
  bool ReadDataFromRosbag(const std::string _bag_path,
                          const std::shared_ptr<SensorParams>& _sensor_params,
                          const double _start_time = 0,
                          const double _bag_durr = -1);
  /**
   * @brief 从Rosbag中读取数据
   *
   * @param _bag_path
   * @param _sensor_params
   * @param _start_time
   * @param _bag_durr
   * @return true
   * @return false
   */
  bool ReadDataFromRosbag2(const std::string _bag_path,
                           const std::shared_ptr<SensorParams>& _sensor_params,
                           const double _start_time = 0,
                           const double _bag_durr = -1);

  /**
   * @brief GetImuData 获取第index帧IMU数据
   * @param imu_data
   * @param index
   * @return
   */
  bool GetImuData(IMUData& _imu_data, int _index);

  /**
   * @brief Get the Imu Data object 获取第index帧IMU数据
   *
   * @param _imu_data
   * @param _index
   * @return true
   * @return false
   */
  bool GetImuData(IMUData2& _imu_data, int _index);

  /**
   * @brief GetImuDataInterval 获取IMU数据区间
   * @param imu_vec
   * @param begin_time
   * @param end_time
   * @return
   */
  bool GetImuDataInterval(std::vector<IMUData>& _imu_vec, double _begin_time,
                          double _end_time);

  /**
   * @brief GetImuDataInterval 获取IMU数据区间
   * @param imu_vec
   * @param begin_time
   * @param end_time
   * @return
   */
  bool GetImuDataInterval(std::vector<IMUData2>& _imu_vec,
                          int64_t _begin_time_ns, int64_t _end_time_ns);

  /**
   * @brief 激光-IMU对齐
   *
   * @param _start_time
   * @param _imu_orientation
   */
  void LidarImuDataAlignment(double& _start_time,
                             Eigen::Quaterniond& _imu_orientation);

  /**
   * @brief 激光-IMU对齐
   *
   * @param _imu_state
   * @param _imu_data
   */
  void LidarImuData2Alignment(IMUState& _imu_state, IMUData2& _imu_data);

  /**
   * @brief Get the Lidar Data Count object 获取激光雷达数据数量
   *
   * @return size_t
   */
  size_t GetLidarDataCount();

  /**
   * @brief Get the Lidar Start Time object 获取激光雷达起始时间
   *
   * @return double
   */
  double GetLidarStartTime();

 private:
  /// @brief 数据转换器
  std::shared_ptr<DataConverter> data_converter_;

  /// @brief IMU数据序列
  std::vector<IMUData> imu_data_sequence_;
  /// @brief IMU2数据序列
  std::vector<IMUData2> imu_data2_sequence_;
  /// @brief 轮速计数据序列
  std::vector<OdometryData> odom_data_sequence_;
  /// @brief GNSS数据序列
  std::vector<GNSSData> gnss_data_sequence_;

  /// @brief 获取IMU数据指针
  size_t get_imu_pointer_;
  /// @brief 获取轮速计数据指针
  size_t get_wheel_pointer_;
  /// @brief 获取GNSS数据指针
  size_t get_gnss_data_pointer_;

  /// @brief 激光数据数量
  size_t lidar_data_count_;
  /// @brief 激光雷达起始时间
  double lidar_start_time_;
};

}  // namespace multi_sensor_mapping

#endif