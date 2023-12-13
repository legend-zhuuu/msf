#ifndef MSM_LIDAR_DETECTOR_H
#define MSM_LIDAR_DETECTOR_H

#include <ros/ros.h>

#include <mutex>
#include <thread>

#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class ParamSet;
class LidarTagDetector;
class LidarDetectionParams;
class SensorParams;
class DataConverter;

/**
 * @brief 激光雷达检测器
 *
 */
class LidarDetector {
 public:
  typedef std::unique_lock<std::mutex> Lock;
  /**
   * @brief Construct a new Lidar Initializer object
   *
   */
  LidarDetector();

  /**
   * @brief Set the Params object 设置参数
   *
   * @param _param_set
   */
  void SetParams(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief  初始化
   *
   */
  void Init();

  /**
   * @brief 开始检测
   *
   */
  void Start();

  /**
   * @brief 结束检测
   *
   */
  void Stop();

  /**
   * @brief 注册tag姿态回调函数
   *
   * @param _tag_poses
   */
  void RegTagPoses(
      const std::function<void(const std::vector<PoseData>&)>& _tag_poses);

  /**
   * @brief 是否在运行
   *
   * @return true
   * @return false
   */
  bool IsRunning();

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
   * @brief CloudHandler 激光点云回调函数
   * @param cloud_msg
   */
  void CloudHandler(const sensor_msgs::PointCloud2::ConstPtr& _cloud_msg);

  /**
   * @brief 重置激光雷达检测器
   *
   */
  void ResetDetector();

 private:
  /// @brief 退出处理标志位
  bool exit_process_flag_;
  /// @brief 初始化成功标志位
  bool init_done_flag_;
  /// @brief 开始标志位
  bool start_flag_;

  /// @brief ros节点
  std::unique_ptr<ros::NodeHandle> nh_;
  /// @brief 激光点云数据订阅器
  ros::Subscriber sub_cloud_;

  /// @brief 传感器参数
  std::shared_ptr<SensorParams> sensor_params_;
  /// @brief 激光检测参数
  std::shared_ptr<LidarDetectionParams> lidar_detection_params_;
  /// @brief 激光tag检测器
  std::shared_ptr<LidarTagDetector> lidar_tag_detector_;

  /// @brief 数据转换器
  std::shared_ptr<DataConverter> data_converter_;

  /// @brief 检测线程
  std::thread detection_thread_;

  /// @brief 点云数据缓存
  std::deque<TimedCloudData> cloud_buffer_;

  /// @brief 互斥锁
  std::mutex lidar_mutex_;

  /// @brief tag姿态回调函数
  std::function<void(const std::vector<PoseData>&)> cb_tag_poses_;
};

}  // namespace multi_sensor_mapping

#endif