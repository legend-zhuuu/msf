#ifndef MSM_PATCHWORK_PARAMS_H
#define MSM_PATCHWORK_PARAMS_H

#include "multi_sensor_mapping/param/param_base.h"

namespace multi_sensor_mapping {

/**
 * @brief Patchwork参数
 *
 */
class PatchWorkParams : public ParamBase {
 public:
  typedef std::shared_ptr<PatchWorkParams> Ptr;

  PatchWorkParams(std::string _name = "patchwork_params") { name_ = _name; }

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
  ParamType Type() const override { return ParamType::PARAM_TYPE_PATCHWORK; }

  /**
   * @brief Name 重写名称返回函数
   */
  std::string Name() const override { return this->name_; }

 public:
  /// @brief 传感器高度
  double sensor_height;
  /// @brief PCA迭代次数.
  int num_iter;
  /// @brief 被选择为最低点的最大点数
  int num_lpr;
  /// @brief 每个patch估计地面的最小点数量
  int num_min_pts;
  /// @brief 用于地面点初始种子选择的最低点代表阈值
  double th_seeds;
  /// @brief 地面厚度的阈值
  double th_dist;
  /// @brief 垂直地面最低点的阈值
  double th_seeds_v;
  /// @brief 最低点地面的阈值
  double th_dist_v;
  /// @brief 地面估计区域的最大距离
  double max_r;
  /// @brief 地面估计区域的最小距离
  double min_r;
  /// @brief 地面似然估计的阈值
  double uprightness_thr;
  /// @brief 使能RNR(反射噪声去除)
  bool enable_RNR;
  /// @brief 使能RVPF(区域垂直平面拟合)
  bool enable_RVPF;
  /// @brief 使能TGR(空间地面恢复)
  bool enable_TGR;
  /// @brief RNR角度阈值
  double RNR_ver_angle_thr;
  /// @brief RNR强度阈值
  double RNR_intensity_thr;
  /// @brief 在adaptive_seed_selection_margin * sensor_height的点被滤除
  double adaptive_seed_selection_margin;
};
}  // namespace multi_sensor_mapping

#endif