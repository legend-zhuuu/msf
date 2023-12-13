#ifndef MSM_TINY_LOCATION_PARAMS_H
#define MSM_TINY_LOCATION_PARAMS_H

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 轻量级定位参数
 *
 */
class TinyLocationParams : public ParamBase {
 public:
  typedef std::shared_ptr<TinyLocationParams> Ptr;

  /**
   * @brief Construct a new Tiny Locator Params object
   *
   * @param _name
   */
  TinyLocationParams(std::string _name = "tiny_location_params");

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
  ParamType Type() const override {
    return ParamType::PARAM_TYPE_TINY_LOCATION;
  }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 自动开始
  bool auto_start;

  /// @brief NDT分辨率
  double target_resolution;
  /// @brief NDT分辨率
  double source_resolution;
  /// @brief 配准间隔时间
  double registration_interval;

  /// @brief 多线程核心数
  int num_cores;

  /// @brief 方向角度偏置(激光朝向与实际方向的偏角)
  float direction_offset;
};
}  // namespace multi_sensor_mapping

#endif