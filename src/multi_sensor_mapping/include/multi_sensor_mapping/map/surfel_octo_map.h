#ifndef MSM_SURFEL_OCTO_MAP_H
#define MSM_SURFEL_OCTO_MAP_H

#include "multi_sensor_mapping/utils/utils_pointcloud.h"
// #include "ufo/map/surfel_map.h"

namespace multi_sensor_mapping {

/**
 * @brief 面元
 *
 */
struct SurfelUnit {
  /// @brief 中心
  Eigen::Vector3d mean;
  /// @brief 法向量
  Eigen::Vector3d normal;
  /// @brief
  Eigen::Vector3d eigen_value;

  /// @brief 朝向(可视化)
  Eigen::Quaterniond direction;
  /// @brief 深度
  int depth_level;
  /// @brief 叶节点分辨率
  double leaf_resolution;
};

/**
 * @brief 基于ufo-map的面元地图
 *
 */
class SurfelOctoMap {
 public:
  /**
   * @brief Construct a new Surfel Map object 构造函数
   *
   * @param _resolution
   * @param _depth_level
   */
  SurfelOctoMap(double _resolution = 0.1, int _depth_level = 16);

  /**
   * @brief 添加点云
   *
   * @param _cloud
   */
  void InsertCloud(const CloudTypePtr& _cloud);

  /**
   * @brief 查询面元
   *
   * @param _query_min_depth
   * @param _query_max_depth
   * @param _min_planarity
   * @return std::vector<SurfelUnit>
   */
  std::vector<SurfelUnit> QuerySurfelUnits(int _query_min_depth,
                                           int _query_max_depth,
                                           double _min_planarity);

  /**
   * @brief 保存
   *
   * @param _path
   */
  void Save(std::string _path);

  /**
   * @brief 加载
   *
   * @param _path
   */
  bool Load(std::string _path);

 private:
  /// @brief 分辨率
  double leaf_resolution_;
  /// @brief 最大深度
  int depth_level_;

  /// @brief 地图
  // std::shared_ptr<ufo::map::SurfelMap> map_core_;
};
}  // namespace multi_sensor_mapping

#endif