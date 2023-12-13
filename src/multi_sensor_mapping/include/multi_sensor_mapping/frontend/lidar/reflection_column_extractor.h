#ifndef MSM_REFLECTION_COLUMN_EXTRACTOR_H
#define MSM_REFLECTION_COLUMN_EXTRACTOR_H

#include <memory>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief 反光柱提取器
 *
 */
class ReflectionColumnExtractor {
 public:
  /**
   * @brief Construct a new Reflection Column Extractor object
   *
   */
  ReflectionColumnExtractor();

  /**
   * @brief Construct a new Reflection Column Extractor object
   *
   * @param _interest_radius
   * @param _column_diamater
   */
  ReflectionColumnExtractor(float _interest_radius, float _column_diamater);

  /**
   * @brief 提取反光柱点云
   *
   * @param _cloud
   * @return std::vector<PointType>
   */
  std::vector<PointType> ExtractColumns(const CloudTypePtr& _cloud);

 private:
  /**
   * @brief Get the Interest Point Cloud object ROI滤波，降低聚类提取错误率
   *
   * @param _cloud_in
   */
  void GetInterestPointCloud(const CloudTypePtr& _cloud_in);

  /**
   * @brief Get the Multi Layer Cloud object 多层聚类提取反光柱
   *
   * @param _search_point
   */
  void GetMultiLayerCloud(const PointType& _search_point);

  /**
   * @brief 对反光柱的单层点云进行中心点提取
   *
   * @param _single_layer_cloud
   * @return PointType
   */
  PointType FindCenterOfSingleLayer(CloudType& _single_layer_cloud);

  /**
   * @brief 通过多层的中心点获得反光柱的中心点
   *
   * @param multi_layer_points 反光柱对应的多层点云 即反光柱点云簇
   * @note 简单的均值滤波和垂直性判断
   */
  PointType FindCenterOfColumnCluster();

 private:
  /// @brief 检测半径
  float interest_radius_;
  /// @brief 反光柱直径 (m)
  float column_diamater_;
  /// @brief 兴趣区域点云
  CloudTypePtr interest_cloud_;
  /// @brief 高反射率点云
  CloudTypePtr high_intensity_cloud_;
  /// @brief KD tree
  pcl::KdTreeFLANN<PointType> kdtree_cloud_;
  /// @brief 多层点云
  std::vector<CloudType> multi_layer_cloud_;
};

}  // namespace multi_sensor_mapping

#endif