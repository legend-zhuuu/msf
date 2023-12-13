#ifndef MSM_CLINS_MAPPER_H
#define MSM_CLINS_MAPPER_H

#include "multi_sensor_mapping/core/lidar_mapper_base.h"
#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

class ClinsCore;
class DataManager;
class ParamSet;
class ClinsParams;

/**
 * @brief 基于连续时间轨迹的激光-IMU融合建图
 *
 */
class ClinsMapper : public LidarMapperBase {
 public:
  /**
   * @brief Construct a new Clins Mapper objects
   *
   */
  ClinsMapper();

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
  void ConvertImuToLidarFrame(IMUData2& imu_data);

 private:
  /// @brief 数据起始时间
  int64_t data_start_time_ns_ = 0;

  /// @brief 建图参数
  std::shared_ptr<ClinsParams> mapper_params_;

  /// @brief 建图核心
  std::shared_ptr<ClinsCore> clins_core_;
  /// @brief 数据管理器
  std::shared_ptr<DataManager> data_manager_;
};

}  // namespace multi_sensor_mapping

#endif