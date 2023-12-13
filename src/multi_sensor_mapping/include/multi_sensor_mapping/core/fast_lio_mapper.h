#ifndef MSM_FAST_LIO_MAPPER_H
#define MSM_FAST_LIO_MAPPER_H

#include <ros/ros.h>

#include <mutex>
#include <thread>

#include "multi_sensor_mapping/core/lidar_mapper_base.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class ParamSet;
class FastLIOCore;
class FastLIOParams;
class WatchDog;

/**
 * @brief FAST-LIO建图 (实时运行)
 *
 */
class FastLIOMapper : public LidarMapperBase {
 public:
  typedef std::unique_lock<std::mutex> Lock;
  /**
   * @brief Construct a new Fast LIO Mapper object
   *
   */
  FastLIOMapper();

  /**
   * @brief Set the Params object
   *
   * @param _param_set
   */
  virtual void SetParams(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief  初始化
   *
   */
  virtual void Init();

  /**
   * @brief 开始里程计
   *
   */
  virtual void Start();

  /**
   * @brief 结束里程计
   *
   */
  virtual void Stop();

  /**
   * @brief 数据是否断连
   *
   * @return true
   * @return false
   */
  bool SensorDisconnected();

 private:
  /**
   * @brief 循环处理
   *
   */
  void LoopProcess();

  /**
   * @brief 注册传感器数据订阅器
   *
   */
  void RegisterSensorSub();

  /**
   * @brief ImuHandler IMU回调函数
   * @param imu_msg
   */
  void ImuHandler(const sensor_msgs::Imu::ConstPtr& _imu_msg);

  /**
   * @brief CloudHandler 激光点云回调函数
   * @param cloud_msg
   */
  void CloudHandler(const sensor_msgs::PointCloud2::ConstPtr& _cloud_msg);

  /**
   * @brief 同步传感器数据
   *
   * @return true
   * @return false
   */
  bool SyncSensorData();

  /**
   * @brief 重置建图器
   *
   */
  void ResetMapper();

  /**
   * @brief 检测传感器触发
   *
   */
  bool CheckSensorTrigger();

 private:
  /// @brief LIO参数
  std::shared_ptr<FastLIOParams> lio_params_;

  /// @brief FAST-LIO核心
  std::shared_ptr<FastLIOCore> fast_lio_core_;

  /// @brief ros节点
  std::unique_ptr<ros::NodeHandle> nh_;
  /// @brief IMU数据订阅器
  ros::Subscriber sub_imu_;
  /// @brief 激光点云数据订阅器
  ros::Subscriber sub_cloud_;
  /// @brief 传感器断连标志位
  bool sensor_disconnected_flag_;

  /// @brief 最新的IMU时间戳
  double last_imu_timestamp_;
  /// @brief IMU数据缓存
  std::deque<IMUData> imu_buffer_;
  /// @brief 点云数据缓存
  std::deque<TimedCloudData> cloud_buffer_;

  /// @brief 当前处理的点云数据
  TimedCloudData current_cloud_data_;
  /// @brief 当前处理的IMU数据
  std::deque<IMUData> current_imu_data_;
  /// @brief 激光平均的扫描时间
  double lidar_mean_scan_time_;
  /// @brief 扫描数量
  int scan_num_;

  /// @brief 激光雷达看门狗
  std::shared_ptr<WatchDog> lidar_watch_dog_;
  /// @brief IMU看门狗
  std::shared_ptr<WatchDog> imu_watch_dog_;

  /// @brief 互斥锁
  std::mutex lidar_mutex_;
  std::mutex imu_mutex_;
};

}  // namespace multi_sensor_mapping

#endif