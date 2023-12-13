/* ----------------------------------------------------------------------------

 * Copyright 2021, APRIL Lab,
 * Hangzhou, Zhejiang, China
 * All Rights Reserved
 * Authors: Hu kewei, Wu Hangyu, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#ifndef MSM_IMU_ROTATION_PREDICTOR_H
#define MSM_IMU_ROTATION_PREDICTOR_H

#include "multi_sensor_mapping/frontend/inertial/ImuPredictorBase.h"

namespace multi_sensor_mapping {

/**
 * @brief The ImuRotationPredictor class 根据IMU的四元数来预测姿态
 */
class ImuRotationPredictor : ImuPredictorBase {
 public:
  typedef std::shared_ptr<ImuRotationPredictor> Ptr;

  /**
   * @brief ImuRotationPredictor 构造函数
   * @param _extrinsic_params
   */
  ImuRotationPredictor(ExtrinsicParams::Ptr _extrinsic_params);

  /**
   * @brief AddImuData 添加IMU测量值
   * @param _imu_data
   */
  void AddImuData(const IMUData& _imu_data);

  /**
   * @brief CorrectPose 加入传感器测量值，修正姿态
   * @param timestamp
   * @param pose
   * @param is_degenerate
   * @param frame
   */
  void CorrectPose(double timestamp, const Transf& pose,
                   bool is_degenerate = false,
                   FrameType frame = FrameType::LIDAR);

  /**
   * @brief Predict 姿态预测
   * @param timestamp
   * @param sensor_pose
   * @param frame
   * @return
   */
  bool Predict(double timestamp, Transf& sensor_pose,
               FrameType frame = FrameType::LIDAR);

 private:
  /**
   * @brief PoseToImuFrame 传感器姿态转到IMU坐标系下
   * @param pose
   * @param frame
   * @return
   */
  Transf PoseToImuFrame(const Transf& pose, FrameType frame = FrameType::LIDAR);

  /**
   * @brief ImuPoseToSensorFrame
   * IMU姿态转换到传感器坐标系下(PoseToImuFrame的逆操作)
   * @param pose
   * @param frame
   * @return
   */
  Transf ImuPoseToSensorFrame(const Transf& pose,
                              FrameType frame = FrameType::LIDAR);

  /**
   * @brief FindImuRotation
   * @param time
   * @return
   */
  Eigen::Quaternionf FindImuRotation(const double time);

 private:
  /// @brief IMU数据的缓存
  std::deque<IMUData> imu_data_cache_;
  /// @brief 最新的姿态
  Transf latest_pose_;
  /// @brief 最新时刻的旋转
  Eigen::Quaternionf quater_at_latest_time_;
  /// @brief 预测标志位
  bool predict_valid_flag_ = false;
};

}  // namespace multi_sensor_mapping

#endif
