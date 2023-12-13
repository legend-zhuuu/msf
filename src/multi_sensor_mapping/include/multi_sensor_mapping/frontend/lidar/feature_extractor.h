
#ifndef MSM_FEATURE_EXTRACTOR_H
#define MSM_FEATURE_EXTRACTOR_H

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "multi_sensor_mapping/utils/utils_voxel_filter.h"

namespace multi_sensor_mapping {

/**
 * @brief The Smoothness struct 曲率
 */
struct Smoothness {
  float value;
  size_t ind;
};

/**
 * @brief The MSM_FEATURE_EXTRACTOR_H class LOAM特征提取器
 * Adapted from LIO-SAM LOAM
 */
class FeatureExtractor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<FeatureExtractor> Ptr;

  /**
   * @brief FeatureExtractor 默认构造函数
   */
  FeatureExtractor();

  /**
   * @brief 构造函数
   *
   * @param _num_ring
   * @param _horizon_scan
   * @param _edge_threshold
   * @param _surf_threshold
   * @param _surf_feature_leaf_size
   */
  FeatureExtractor(int _num_ring, int _horizon_scan, double _edge_threshold,
                   double _surf_threshold, double _surf_feature_leaf_size);

  /**
   * @brief Destroy the Feature Extractor object
   *
   */
  ~FeatureExtractor();

  /**
   * @brief SetParams 设置参数
   * @param _params
   */
  void SetParams(int _num_ring, int _horizon_scan, double _edge_threshold,
                 double _surf_threshold, double _surf_feature_leaf_size);

  /**
   * @brief ExtractLOAMFeature 提取LOAM特征(外部调用)
   * @param _full_cloud
   * @param _corner_cloud
   * @param _surface_cloud
   */
  void ExtractLOAMFeature(VelRTPointCloudPtr& _full_cloud,
                          VelRTPointCloudPtr& _corner_cloud,
                          VelRTPointCloudPtr& _surface_cloud);

  /**
   * @brief ExtractLOAMFeature 提取LOAM特征(外部调用)
   * @param _full_cloud
   * @param _corner_cloud
   * @param _surface_cloud
   */
  void ExtractLOAMFeature(VelRTPointCloudPtr& _full_cloud,
                          CloudTypePtr& _corner_cloud,
                          CloudTypePtr& _surface_cloud);

 private:
  /**
   * @brief AllocateMemory 分配参数内存
   */
  void AllocateMemory();

  /**
   * @brief ProjectPointCloud 将点云投影到range mat上
   * @param _cloud
   * @param dist_image
   * @param corresponding_cloud
   */
  void ProjectPointCloud(const VelRTPointCloudPtr& _cloud, cv::Mat& dist_image,
                         VelRTPointCloudPtr& corresponding_cloud);

  /**
   * @brief CloudExtraction
   */
  void CloudExtraction(VelRTPointCloudPtr& _cloud);

  /**
   * @brief CaculateSmoothness 计算曲率
   */
  void CaculateSmoothness();

  /**
   * @brief MarkOccludedPoints 剔除坏点
   */
  void MarkOccludedPoints();

  /**
   * @brief ExtractFeatures 角点和平面点特征提取
   * @param _corner_cloud
   * @param _surface_cloud
   */
  void ExtractFeatures(VelRTPointCloudPtr& _corner_cloud,
                       VelRTPointCloudPtr& _surface_cloud);

  /**
   * @brief CloudConvert 点云类型转换
   * @param intput
   * @param output
   */
  void CloudConvert(VelRTPointCloudPtr& input, CloudTypePtr& output);

 private:
  /// @brief 激光线数
  int num_ring_;
  /// @brief 激光扫描数
  int horizon_scan_;
  /// @brief 角点特征
  double edge_threshold_;
  /// @brief 平面点特征
  double surf_threshold_;
  /// @brief 平面特征降采样
  double surf_feature_leaf_size_;

  /// @brief 保存每个点距离的数组
  std::vector<float> point_range_list_;
  /// @brief 保存每个点的列索引
  std::vector<int> point_column_id_;
  /// @brief 每个scan起始点索引
  std::vector<int> start_ring_index_;
  /// @brief 每个scan结束点索引
  std::vector<int> end_ring_index_;
  /// @brief 保存每个点的距离
  cv::Mat range_mat_;
  /// @brief 体素滤波器
  utils::VoxelFilter<VelRTPoint> down_size_filter_;
  /// @brief 保存点云曲率
  std::vector<Smoothness> cloud_smoothness_;
  /// @brief 保存曲率
  float* cloud_curvature_;
  /// @brief 保存点云邻域选择情况
  int* cloud_neighbor_picked_;
  /// @brief 保存点云标签
  int* cloud_label_;
  /// @brief 提取后的点云
  VelRTPointCloudPtr extracted_cloud_ptr_;
};

}  // namespace multi_sensor_mapping

#endif
