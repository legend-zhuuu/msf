#ifndef MSM_EXTRINSIC_PARAMS_H
#define MSM_EXTRINSIC_PARAMS_H

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief The ExtrinsicParams class 外参参数
 */
class ExtrinsicParams : public ParamBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<ExtrinsicParams> Ptr;

  ExtrinsicParams(std::string _name = "extrinsic_params")
      : imu_to_baselink(Eigen::Matrix4d::Identity()),
        wheel_to_baselink(Eigen::Matrix4d::Identity()),
        gnss_to_baselink(Eigen::Matrix4d::Identity()) {
    name_ = _name;
  }

 public:
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
  ParamType Type() const override { return ParamType::PARAM_TYPE_EXTRINSIC; }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

  /**
   * @brief 打印单个传感器外参
   *
   * @param _sensor_name
   * @param _matrix
   */
  void PrintSensorExtParam(std::string _sensor_name,
                           const Eigen::Matrix4d& _matrix);

  /**
   * @brief 获取激光雷达的旋转
   *
   * @param _lidar_name
   * @return Eigen::Quaterniond
   */
  Eigen::Quaterniond GetLidarToBaseRotation(const LidarName& _lidar_name);

 public:
  /// @brief 激光雷达外参
  std::map<LidarName, Eigen::Matrix4d> lidar_to_baselink;
  /// @brief IMU外参
  Eigen::Matrix4d imu_to_baselink;
  /// @brief 轮速计外参
  Eigen::Matrix4d wheel_to_baselink;
  /// @brief GNSS外参
  Eigen::Matrix4d gnss_to_baselink;

  /// @brief IMU平移外参
  Eigen::Vector3d imu_to_baselink_translation_;
  /// @brief IMU旋转外参
  Eigen::Quaterniond imu_to_baselink_rotation_;
  /// @brief 轮速计平移外参
  Eigen::Vector3d wheel_to_baselink_translation_;
  /// @brief 轮速计旋转外参
  Eigen::Quaterniond wheel_to_baselink_rotation_;
  /// @brief 轮速计平移外参
  Eigen::Vector3d gnss_to_baselink_translation_;
  /// @brief 轮速计旋转外参
  Eigen::Quaterniond gnss_to_baselink_rotation_;
};

}  // namespace multi_sensor_mapping

#endif
