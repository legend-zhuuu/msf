#ifndef MSM_IMU_POSE_PREDICTOR_H
#define MSM_IMU_POSE_PREDICTOR_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "multi_sensor_mapping/frontend/imu/imu_predictor_base.h"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace multi_sensor_mapping {

/**
 * @brief The ImuPosePredictor class 基于IMU预积分的姿态预测器
 * Adapted from LIO-SAM
 */
class ImuPosePredictor : public ImuPredictorBase {
 public:
  typedef std::shared_ptr<ImuPosePredictor> Ptr;

  /**
   * @brief ImuPosePredictor 构造函数
   * @param _extrinsic_params
   * @param _imu_gravity
   * @param _imu_accel_noise
   * @param _imu_gyro_noise
   * @param _imu_accel_bias_noise
   * @param _imu_gyro_bias_noise
   */
  ImuPosePredictor(const std::shared_ptr<ExtrinsicParams>& _extrinsic_params,
                   double _imu_gravity, double _imu_accel_noise,
                   double _imu_gyro_noise, double _imu_accel_bias_noise,
                   double _imu_gyro_bias_noise);

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
  void CorrectPose(double timestamp, const Eigen::Matrix4d& pose,
                   bool is_degenerate = false,
                   FrameType frame = FrameType::LIDAR);

  /**
   * @brief SetInitialBias 设置初始bias
   * @param gyro_bias
   * @param accel_vias
   */
  void SetInitialBias(Eigen::Vector3d gyro_bias, Eigen::Vector3d accel_vias);

  /**
   * @brief Predict 姿态预测
   * @param timestamp
   * @param sensor_pose
   * @param frame
   * @return
   */
  bool Predict(double timestamp, Eigen::Matrix4d& sensor_pose,
               FrameType frame = FrameType::LIDAR);

 private:
  /**
   * @brief InitGtsamVariable 初始化GTSAM变量
   */
  void InitGtsamVariable();

  /**
   * @brief ResetOptimization 重置优化参数
   */
  void ResetOptimization();

  /**
   * @brief PoseToImuFrame 传感器姿态转到IMU坐标系下
   * @param pose
   * @param frame
   * @return
   */
  gtsam::Pose3 PoseToImuFrame(const Eigen::Matrix4d& pose,
                              FrameType frame = FrameType::LIDAR);

  /**
   * @brief ImuPoseToSensorFrame
   * IMU姿态转换到传感器坐标系下(PoseToImuFrame的逆操作)
   * @param pose
   * @param frame
   * @return
   */
  Eigen::Matrix4d ImuPoseToSensorFrame(const gtsam::Pose3& pose,
                                       FrameType frame = FrameType::LIDAR);

  /**
   * @brief FailureDetection 优化失败检测
   * @param velCur
   * @param biasCur
   * @return
   */
  bool FailureDetection(const gtsam::Vector3& velCur,
                        const gtsam::imuBias::ConstantBias& biasCur);

 private:
  /// @brief 重力
  double imu_gravity_;
  /// @brief IMU 加速度噪声
  double imu_accel_noise_;
  /// @brief IMU 角速度噪声
  double imu_gyro_noise_;
  /// @brief IMU 加速度bias的噪声
  double imu_accel_bias_noise_;
  /// @brief IMU 角速度bias的噪声
  double imu_gyro_bias_noise_;

  /// @brief 初始化标志位
  bool initialized_flag_;
  /// @brief
  int key_;
  /// @brief 噪声模型
  gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr correction_noise2_;
  gtsam::Vector noise_model_between_bias_;

  /// @brief IMU数据的缓存
  std::deque<IMUData> imu_data_cache_;
  std::deque<IMUData> opt_imu_data_cache_;

  /// @brief IMU 预积分
  gtsam::PreintegratedImuMeasurements* imu_integrator_;
  gtsam::PreintegratedImuMeasurements* imu_integrator_opt_;

  /// @brief 优化器
  gtsam::ISAM2 optimizer_;
  /// @brief 因子图
  gtsam::NonlinearFactorGraph graph_factors_;
  /// @brief 因子图的节点值
  gtsam::Values graph_values_;

  /// @brief 上个状态
  gtsam::Pose3 prev_pose_;
  gtsam::Vector3 prev_vel_;
  gtsam::NavState prev_state_;
  gtsam::imuBias::ConstantBias prev_bias_;

  /// @brief 手动设置初始IMU标志位
  bool initial_bias_flag_;
  /// @brief 上一个优化的IMU时间
  double last_opt_imu_time_;
  /// @brief 最新的矫正时间
  double last_correction_time_;
};

}  // namespace multi_sensor_mapping

#endif
