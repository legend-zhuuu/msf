#ifndef MSM_SDK_IMU_INITIALIZER_H
#define MSM_SDK_IMU_INITIALIZER_H

#include <functional>
#include <memory>
#include <thread>

#include "msm_types.h"

namespace multi_sensor_mapping {
class ImuInitializer;
}

namespace msm_sdk {

/**
 * @brief IMU初始化API
 *
 */
class APIImuInitializer {
 public:
  /**
   * @brief 构造函数
   *
   */
  APIImuInitializer();

  /**
   * @brief 初始化
   *
   */
  void Init();

  /**
   * @brief 初始化
   *
   */
  void Init(double _excite_threshold);

  /**
   * @brief 开始
   *
   */
  void Start();

  /**
   * @brief 结束
   *
   */
  void Stop();

  /**
   * @brief 输入IMU数据
   *
   * @param _data
   */
  void InputIMU(const IMU& _data);

  /**
   * @brief 注册回调函数
   *
   * @param _cb
   */
  void RegStationaryCoeffCallback(
      const std::function<void(const bool&, const double&)>& _cb);

 private:
  /**
   * @brief 核心处理函数
   *
   */
  void Process();

 private:
  /// @brief IMU初始化
  std::shared_ptr<multi_sensor_mapping::ImuInitializer> imu_initializer_ptr_;
  /// @brief 获取静止参数回调函数
  std::function<void(const bool&, const double&)> cb_put_stationary_coeff_;
  /// @brief 处理线程
  std::thread process_thread_;
};
}  // namespace msm_sdk

#endif