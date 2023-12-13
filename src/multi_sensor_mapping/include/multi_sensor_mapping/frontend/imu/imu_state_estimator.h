#ifndef MSM_IMU_STATE_ESTIMATOR_H
#define MSM_IMU_STATE_ESTIMATOR_H

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"
#include "multi_sensor_mapping/utils/utils_eigen.h"

namespace multi_sensor_mapping {

/**
 * @brief IMU状态估计器
 *
 */
class ImuStateEstimator {
 public:
  typedef std::shared_ptr<ImuStateEstimator> Ptr;

  /**
   * @brief Construct a new Imu State Estimator object
   *
   */
  ImuStateEstimator();

  /**
   * @brief 传入IMU数据
   *
   * @param _imu_data
   */
  void FeedImuData(const IMUData2& _imu_data);

  /**
   * @brief 积分
   *
   * @param _imu_state
   * @param _from_timestamp
   * @param _to_timestamp
   */
  void Propagate(const IMUState& _imu_state, int64_t _from_timestamp,
                 int64_t _to_timestamp);

  /**
   * @brief Get the Propagate Start State object 获取积分状态
   *
   * @return const IMUState&
   */
  const IMUState& GetPropagateStartState() const {
    return propagate_start_state_;
  }

  /**
   * @brief Get the Latest State object 获取最新状态
   *
   * @return const IMUState&
   */
  const IMUState& GetLatestState() const { return latest_state_; }

 private:
  /// @brief IMU数据缓存
  Eigen::aligned_vector<IMUData2> imu_data_cache_;
  /// @brief IMU开始状态
  IMUState propagate_start_state_;
  /// @brief 最新状态
  IMUState latest_state_;
};

}  // namespace multi_sensor_mapping

#endif