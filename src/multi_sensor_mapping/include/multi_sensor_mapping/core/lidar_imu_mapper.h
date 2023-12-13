#ifndef MSM_LIDAR_IMU_MAPPER_H
#define MSM_LIDAR_IMU_MAPPER_H

#include "multi_sensor_mapping/core/lidar_mapper_base.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class LidarImuMappingCore;
class DataManager;
class LidarImuMappingParams;
class ParamSet;
class FastLIOCore;
class FastLIOParams;

/**
 * @brief 激光-IMU建图
 *
 */
class LidarImuMapper : public LidarMapperBase {
 public:
  /**
   * @brief Construct a new Lidar Imu Mapper object 构造函数
   *
   */
  LidarImuMapper();

  /**
   * @brief Set the Params object 设置参数
   *
   * @param _param_set
   */
  virtual void SetParams(const std::shared_ptr<ParamSet>& _param_set);

  /**
   * @brief 初始化
   *
   */
  virtual void Init();

  /**
   * @brief 开始定位
   *
   */
  virtual void Start();

  /**
   * @brief 结束定位
   *
   */
  virtual void Stop();

 private:
  /**
   * @brief 建图核心处理函数
   *
   */
  bool ProcessMapping();

  /**
   * @brief 静止初始化处理函数
   *
   * @return true
   * @return false
   */
  bool StaticInitializationProcess();

  /**
   * @brief IMU初始化处理
   *
   * @return true
   * @return false
   */
  bool ImuInitializationProcess();

  /**
   * @brief 循环处理
   *
   */
  void LoopProcess();

  /**
   * @brief IMU坐标转换
   *
   * @param imu_data
   */
  void ConvertImuToLidarFrame(IMUData& imu_data);

  /**
   * @brief 检查是否为关键帧
   *
   * @param _pose
   * @return true
   * @return false
   */
  bool CheckKeyScan(const PoseData& _pose);

 private:
  /// @brief 建图参数
  std::shared_ptr<LidarImuMappingParams> mapper_params_;
  /// @brief LIO参数
  std::shared_ptr<FastLIOParams> lio_params_;

  /// @brief 建图核心
  std::shared_ptr<LidarImuMappingCore> lidar_imu_mapping_core_;

  /// @brief FAST-LIO核心
  std::shared_ptr<FastLIOCore> fast_lio_core_;

  /// @brief 数据管理器
  std::shared_ptr<DataManager> data_manager_;
};
}  // namespace multi_sensor_mapping

#endif