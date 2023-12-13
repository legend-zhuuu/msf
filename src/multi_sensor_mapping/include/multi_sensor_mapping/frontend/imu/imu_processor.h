#ifndef MSM_IMU_PROCESSOR_H
#define MSM_IMU_PROCESSOR_H

#include <memory>
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/use_ikfom.h"

#define MAX_INI_COUNT (50)

namespace multi_sensor_mapping {

/**
 * @brief IMU状态
 *
 */
struct IMUState6D {
  IMUState6D(double _offset_time, const Eigen::Vector3d& _position,
             const Eigen::Matrix3d& _rotation, const Eigen::Vector3d& _acc,
             const Eigen::Vector3d& _gyr, const Eigen::Vector3d& _vel)
      : offset_time(_offset_time),
        position(_position),
        rotation(_rotation),
        acc(_acc),
        gyr(_gyr),
        vel(_vel) {}

  double offset_time;
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyr;
  Eigen::Vector3d vel;
};

/**
 * @brief IMU处理器
 *
 */
class ImuProcessor {
 public:
  /**
   * @brief Construct a new Imu Processor object
   *
   */
  ImuProcessor();

  /**
   * @brief Reset
   *
   */
  void Reset();

  /**
   * @brief Reset
   *
   * @param _start_timestamp
   * @param _last_imu
   */
  void Reset(const double& _start_timestamp, const IMUData& _last_imu);

  /**
   * @brief Set the Extrinsic object 设置外参
   *
   * @param _translation_lidar2imu
   * @param _rotation_lidar2imu
   */
  void SetExtrinsic(const Eigen::Vector3d& _translation_lidar2imu,
                    const Eigen::Matrix3d& _rotation_lidar2imu);

  /**
   * @brief Set the Gyr Cov object
   *
   * @param _scaler
   */
  void SetGyrCov(const Eigen::Vector3d& _scaler);

  /**
   * @brief Set the Acc Cov object
   *
   * @param _scaler
   */
  void SetAccCov(const Eigen::Vector3d& _scaler);

  /**
   * @brief Set the Gyr Bias Cov object
   *
   * @param _b_g
   */
  void SetGyrBiasCov(const Eigen::Vector3d& _b_g);

  /**
   * @brief Set the Acc Bias Cov object
   *
   * @param _b_a
   */
  void SetAccBiasCov(const Eigen::Vector3d& _b_a);

  /**
   * @brief Process 处理
   *
   * @param _cloud
   * @param _imu_data_vec
   * @param _kf_state
   * @param _feature_undistort
   */
  void Process(const TimedCloudData& _cloud,
               const std::deque<IMUData>& _imu_data_vec,
               esekfom::esekf<state_ikfom, 12, input_ikfom>& _kf_state,
               CloudTypePtr& _feature_undistort);

  /**
   * @brief IMU初始化
   *
   * @param _cloud
   * @param _imu_data_vec
   * @param _kf_state
   * @param _N
   */
  void ImuInit(const TimedCloudData& _cloud,
               const std::deque<IMUData>& _imu_data_vec,
               esekfom::esekf<state_ikfom, 12, input_ikfom>& _kf_state,
               int& _N);

  /**
   * @brief 基于IMU矫正特征点云
   *
   * @param _cloud
   * @param _imu_data_vec
   * @param _kf_state
   * @param _feature_undistort
   */
  void UndistortFeature(const TimedCloudData& _cloud,
                        const std::deque<IMUData>& _imu_data_vec,
                        esekfom::esekf<state_ikfom, 12, input_ikfom>& _kf_state,
                        CloudTypePtr& _feature_undistort);

 public:
  Eigen::Matrix<double, 12, 12> Q_;

  /// @brief 协方差
  Eigen::Vector3d cov_acc_;
  Eigen::Vector3d cov_gyr_;
  Eigen::Vector3d cov_acc_scale_;
  Eigen::Vector3d cov_gyr_scale_;
  Eigen::Vector3d cov_bias_gyr_;
  Eigen::Vector3d cov_bias_acc_;
  ros::Publisher vel_pub;

  /// @brief 雷达时间
  double first_lidar_time_;

 private:
  int init_iter_num_;
  /// @brief 是否是第一帧
  bool first_frame_flag_;
  /// @brief IMU是否需要初始化
  bool imu_need_init_flag_;
  /// @brief 起始时间戳
  double start_timestamp_;
  /// @brief 激光雷达-IMU的外参
  Eigen::Vector3d translation_lidar2imu_;
  Eigen::Matrix3d rotation_lidar2imu_;

  /// @brief 加速度均值
  Eigen::Vector3d mean_acc_;
  /// @brief 陀螺仪均值
  Eigen::Vector3d mean_gyr_;

  Eigen::Vector3d angvel_last_;
  Eigen::Vector3d acc_s_last_;

  double last_lidar_end_time_;
  /// @brief 上一帧IMU数据
  IMUData last_imu_;
  /// @brief IMU数据缓存
  std::deque<IMUData> imu_data_cache_;
  /// @brief IMU状态缓存
  std::vector<IMUState6D> imu_state_container_;
};

}  // namespace multi_sensor_mapping

#endif