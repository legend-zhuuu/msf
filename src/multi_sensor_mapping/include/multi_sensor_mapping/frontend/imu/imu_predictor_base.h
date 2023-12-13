#ifndef MSM_IMU_PREDICTOR_H
#define MSM_IMU_PREDICTOR_H

#include <unordered_map>

#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

/**
 * @brief The ImuPredictorBase class IMU预测器基类
 */
class ImuPredictorBase {
 public:
  typedef std::shared_ptr<ImuPredictorBase> Ptr;

  ImuPredictorBase(const ExtrinsicParams::Ptr& _extrinsic_params) {
    InitExtrinsicParams(_extrinsic_params);
  }

  /**
   * @brief AddImuData 添加IMU测量值
   * @param _imu_data
   */
  virtual void AddImuData(const IMUData& _imu_data) = 0;

  /**
   * @brief CorrectPose 加入传感器测量值，修正姿态
   * @param timestamp
   * @param pose
   * @param is_degenerate
   * @param frame
   */
  virtual void CorrectPose(double timestamp, const Eigen::Matrix4d& pose,
                           bool is_degenerate = false,
                           FrameType frame = FrameType::LIDAR) = 0;

  /**
   * @brief Predict 姿态预测
   * @param timestamp
   * @param sensor_pose
   * @param frame
   * @return
   */
  virtual bool Predict(double timestamp, Eigen::Matrix4d& sensor_pose,
                       FrameType frame = FrameType::LIDAR) = 0;

 protected:
  /**
   * @brief InitExtrinsicParams 初始化外参
   * @param param
   */
  void InitExtrinsicParams(const ExtrinsicParams::Ptr& param) {
    /// Notice: IMU数据经过旋转处理，已经旋转到和LiDAR坐标系的旋转一致
    // TODO 先处理单激光雷达情况，后续再处理多激光雷达建图
    Eigen::Matrix4d fake_base_to_imu =
        param->lidar_to_baselink[LidarName::LIDAR_A];
    fake_base_to_imu.block<3, 1>(0, 3) =
        param->imu_to_baselink.block<3, 1>(0, 3);
    extrinsic_matrix_[FrameType::IMU] = Eigen::Matrix4d::Identity();
    extrinsic_matrix_[FrameType::LIDAR] =
        fake_base_to_imu.inverse() *
        param->lidar_to_baselink[LidarName::LIDAR_A];
    extrinsic_matrix_[FrameType::ODOM] = fake_base_to_imu.inverse();
  }

 protected:
  /// @brief 存放外参 Sensor到IMU坐标系（经常调用，方便查询）
  std::unordered_map<FrameType, Eigen::Matrix4d> extrinsic_matrix_;
};

}  // namespace multi_sensor_mapping
#endif
