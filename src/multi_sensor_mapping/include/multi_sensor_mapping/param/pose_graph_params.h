#ifndef MSM_POSE_GRAPH_PARAMS_H
#define MSM_POSE_GRAPH_PARAMS_H

#include <vector>

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief 位姿图优化参数
 *
 */
class PoseGraphParams : public ParamBase {
 public:
  typedef std::shared_ptr<PoseGraphParams> Ptr;

  /**
   * @brief Construct a new Pose Graph Params object
   *
   * @param _name
   */
  PoseGraphParams(std::string _name = "pose_graph_params");

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
  ParamType Type() const override { return ParamType::PARAM_TYPE_POSE_GRAPH; }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 闭环搜索距离
  double loop_closure_searching_distance;
  /// @brief 闭环距离阈值
  double loop_closure_distance_interval;
  /// @brief 闭环分数阈值
  double loop_closure_score_threshold;
  /// @brief 单层运行
  bool mono_layer_motion;

  /// @brief 地图分辨率
  double map_resolution;
  /// @brief 输出详细信息
  bool output_detail_info;
  /// @brief 输出栅格地图
  bool output_grid_map;
};
}  // namespace multi_sensor_mapping

#endif