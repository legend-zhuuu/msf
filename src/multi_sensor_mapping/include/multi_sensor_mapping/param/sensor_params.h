#ifndef MSM_SENSOR_PARAMS_H
#define MSM_SENSOR_PARAMS_H

#include <vector>

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 激光参数
 *
 */
struct LidarParam {
  LidarName name;
  std::string topic;
  std::string frame_id;
  LidarDataFormat format;
  SensorPurpose purpose;
  bool use_flag = false;
  bool major_flag = false;
};

/**
 * @brief 传感器参数
 *
 */
struct SensorParam {
  std::string topic;
  std::string frame_id;
  bool use_flag = false;
};

class SensorParams : public ParamBase {
 public:
  typedef std::shared_ptr<SensorParams> Ptr;

  SensorParams(std::string _name = "sensor_params") { name_ = _name; }

  /**
   * @brief Load 重写基类加载函数
   * @param _path_of_yaml
   */
  void Load(std::string _path_of_yaml) override;

  /**
   * @brief Print 重写基类打印函数
   */
  void Print() override;

  /**
   * @brief Type 重写类型返回函数
   */
  ParamType Type() const override { return ParamType::PARAM_TYPE_SENSOR; }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

  /**
   * @brief 获取主激光雷达Topic
   *
   * @return std::string
   */
  std::string MajorLidarTopic();

  /**
   * @brief 获取主激光雷达格式
   *
   * @return LidarDataFormat
   */
  LidarDataFormat MajorLidarFormat();

  /**
   * @brief 获取lio激光雷达Topic
   *
   * @return std::string
   */
  std::string LIOLidarTopic();

  /**
   * @brief 获取lio激光雷达格式
   *
   * @return LidarDataFormat
   */
  LidarDataFormat LIOLidarFormat();

 public:
  /// @brief 激光参数
  std::vector<LidarParam> lidar_param;
  /// @brief IMU参数
  SensorParam imu_param;
  /// @brief 轮速计参数
  SensorParam wheel_param;
  /// @brief gnss参数
  SensorParam gnss_param;
  /// @brief 主激光雷达
  LidarName major_lidar_;
};

}  // namespace multi_sensor_mapping

#endif
