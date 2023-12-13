#ifndef MSM_PARAM_SET_H
#define MSM_PARAM_SET_H

#include <vector>

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

class SensorParams;
class ExtrinsicParams;
class LidarImuMappingParams;
class PatchWorkParams;
class ClinsParams;
class FastLIOParams;
class PoseGraphParams;
class TinyLocationParams;
class LidarDetectionParams;

/**
 * @brief 参数集
 *
 */
class ParamSet {
 public:
  /**
   * @brief Construct a new Param Set object 构造函数
   *
   */
  ParamSet();

  /**
   * @brief 加载参数
   *
   * @param _path
   * @return true
   * @return false
   */
  bool Load(std::string _path);

  /**
   * @brief 打印所有参数
   *
   */
  void PrintAll();

  /**
   * @brief Get the Sensor Params object 获取传感器参数
   *
   * @return std::shared_ptr<SensorParams>
   */
  std::shared_ptr<SensorParams> GetSensorParams();

  /**
   * @brief Get the Extrinsic Params object 获取外参参数
   *
   * @return std::shared_ptr<ExtrinsicParams>
   */
  std::shared_ptr<ExtrinsicParams> GetExtrinsicParams();

  /**
   * @brief Get the Lidar Imu Mapping Params object 获取建图参数
   *
   * @return std::shared_ptr<LidarImuMappingParams>
   */
  std::shared_ptr<LidarImuMappingParams> GetLidarImuMappingParams();

  /**
   * @brief Get the Patchwork Params object 获取Patchwork++参数
   *
   * @return std::shared_ptr<PatchWorkParams>
   */
  std::shared_ptr<PatchWorkParams> GetPatchworkParams();

  /**
   * @brief Get the Clins Params object 获取Clins参数
   *
   * @return std::shared_ptr<ClinsParams>
   */
  std::shared_ptr<ClinsParams> GetClinsParams();

  /**
   * @brief Get the Fast LIO Params object 获取Fast LIO参数
   *
   * @return std::shared_ptr<FastLIOParams>
   */
  std::shared_ptr<FastLIOParams> GetFastLIOParams();

  /**
   * @brief Get the Pose Graph Params object 获取位姿图参数
   *
   * @return std::shared_ptr<PoseGraphParams>
   */
  std::shared_ptr<PoseGraphParams> GetPoseGraphParams();

  /**
   * @brief Get the Tiny Location Params object 获取定位参数
   *
   * @return std::shared_ptr<TinyLocationParams>
   */
  std::shared_ptr<TinyLocationParams> GetTinyLocationParams();

  /**
   * @brief Get the Lidar Detection Params object 获取激光检测参数
   *
   * @return std::shared_ptr<LidarDetectionParams>
   */
  std::shared_ptr<LidarDetectionParams> GetLidarDetectionParams();

 public:
  /// @brief 建图模式
  MappingMode mapping_mode_;
  /// @brief 参数序列
  std::map<ParamType, std::shared_ptr<ParamBase>> params_;
};

}  // namespace multi_sensor_mapping

#endif