
#ifndef MSM_LOAM_SCAN_MATCHER_H
#define MSM_LOAM_SCAN_MATCHER_H

#include <Eigen/Eigen>
#include <memory>

#include "multi_sensor_mapping/utils/utils_common.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"

namespace multi_sensor_mapping {

/**
 * @brief The LOAMScanMatcher class 基于LOAM特征的点云匹配
 * Adapted from LIO-SAM LOAM
 */
class LOAMScanMatcher {
 public:
  typedef std::shared_ptr<LOAMScanMatcher> Ptr;

  /**
   * @brief The CornerAssociation struct 角点优化关联
   */
  struct CornerAssociation {
    Eigen::Vector3d ori_point;
    Eigen::Vector3d point_a;
    Eigen::Vector3d point_b;
  };

  /**
   * @brief The SurfaceAssociation struct 平面优化关联
   */
  struct SurfaceAssociation {
    Eigen::Vector3d ori_point;
    Eigen::Vector3d plane_norm;
    double plane_dis;
  };

  /**
   * @brief LOAMScanMatcher 构造函数
   * @param _corner_ds_leaf_size
   * @param _surface_ds_leaf_size
   * @param _corner_feature_min_valid_num
   * @param _surface_feature_min_valid_num
   * @param _num_cores
   * @param _match_method
   */
  LOAMScanMatcher(double _corner_ds_leaf_size, double _surface_ds_leaf_size,
                  int _corner_feature_min_valid_num,
                  int _surface_feature_min_valid_num, int _num_cores,
                  MatchMethod _match_method = MatchMethod::MATCH_METHOD_LOAM);

  /**
   * @brief SetSourceFeature 设置源特征点云
   * @param _corner_cloud
   * @param _surface_cloud
   */
  void SetSourceFeature(CloudTypePtr &_corner_cloud,
                        CloudTypePtr &_surface_cloud);

  /**
   * @brief SetSourceFeature 设置源特征点云
   * @param _corner_cloud
   * @param _surface_cloud
   */
  void SetSourceFeature(VelRTPointCloudPtr &_corner_cloud,
                        VelRTPointCloudPtr &_surface_cloud);

  /**
   * @brief SetTargetFeature 设置目标点云
   * @param _target_corner_cloud
   * @param _target_surface_cloud
   */
  void SetTargetFeature(CloudTypePtr &_target_corner_cloud,
                        CloudTypePtr &_target_surface_cloud);

  /**
   * @brief Align  点云配准
   * @param _prediction
   * @param _result
   */
  bool Align(const Eigen::Matrix4d &_prediction, Eigen::Matrix4d &_result);

  /**
   * @brief isDegenerate 是否退化
   * @return
   */
  inline bool isDegenerate() { return is_degenerate_; }

  /**
   * @brief GetTargetFeatureCloud 获取目标点云
   * @return
   */
  CloudTypePtr GetTargetFeatureCloud();

 private:
  /**
   * @brief DownsampleCloud 降采样点云
   * @param cloud_in
   * @param cloud_out
   * @param is_corner
   */
  void DownsampleCloud(CloudTypePtr cloud_in, CloudTypePtr cloud_out,
                       bool is_corner = true);

  /**
   * @brief CloudConvert 点云类型转换
   * @param intput
   * @param output
   */
  void CloudConvert(VelRTPointCloudPtr &input, CloudTypePtr &output);

  /**
   * @brief UpdateInitialGuess 更新预测姿态
   * @param _prediction
   */
  void UpdateInitialGuess(const Eigen::Matrix4d &_prediction);

  /**
   * @brief Trans2Affine3f
   * @param transformIn
   * @return
   */
  Eigen::Affine3f Trans2Affine3f(float transformIn[]);

  /**
   * @brief UpdatePointAssociateToMap 更新优化后的姿态
   */
  void UpdatePointAssociateToMap();

  /**
   * @brief PointAssociateToMap 点转化到全局坐标系
   * @param pi
   * @param po
   */
  void PointAssociateToMap(PointType pi, PointType &po);

  /**
   * @brief Scan2MapOptimization 当前帧与局部地图匹配
   */
  void Scan2MapOptimization();

  /**
   * @brief FastScan2MapOptimization 当前帧与局部地图快速匹配
   */
  void FastScan2MapOptimization();

  /**
   * @brief CornerOptimization corner 特征构建优化
   */
  void CornerOptimization();

  /**
   * @brief SurfaceOptimazation surface 特征构建优化
   */
  void SurfaceOptimazation();

  /**
   * @brief CombineOptimazationCoeffs 合并角点和平面的优化
   */
  void CombineOptimazationCoeffs();

  /**
   * @brief LMOptimization LM优化
   * @param iter_count
   * @return
   */
  bool LMOptimization(int iter_count);

  /**
   * @brief CornerFeatureAssociation 角点关联
   */
  void CornerFeatureAssociation();

  /**
   * @brief SurfaceFeatureAssociation 平面点关联
   */
  void SurfaceFeatureAssociation();

  /**
   * @brief JointOptimization 联合优化
   */
  bool JointOptimization(int max_iter_num = 4);

  /**
   * @brief CheckConverge 检测是否收敛
   * @return
   */
  bool CheckConverge(int iter_count);

 private:
  /// @brief 匹配方式
  MatchMethod match_method_;
  /// @brief corner点云降采样参数
  double corner_ds_leaf_size_;
  /// @brief surface点云降采样参数
  double surface_ds_leaf_size_;
  /// @brief 当前帧提取的最少角点特征
  int corner_feature_min_valid_num_;
  /// @brief 当前帧提取的最少平面特征
  int surface_feature_min_valid_num_;
  /// @brief OMP所需要的线程数
  int num_cores_;
  /// @brief 配准发生退化
  bool is_degenerate_;

  /// @brief corner特征的kdtree
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_;
  /// @brief surface特征kdtree
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_surface_;
  /// @brief 目标corner特征
  CloudTypePtr target_corner_cloud_;
  /// @brief 目标surface特征
  CloudTypePtr target_surface_cloud_;
  /// @brief 源corner特征
  CloudTypePtr source_corner_cloud_;
  /// @brief 源surface特征
  CloudTypePtr source_surface_cloud_;
  /// @brief 源corner特征（降采样）
  CloudTypePtr source_corner_cloud_ds_;
  /// @brief 源surface特征（降采样）
  CloudTypePtr source_surface_cloud_ds_;

  /// @brief corner特征的体素滤波器
  pcl::VoxelGrid<PointType> corner_ds_filter_;
  /// @brief surface特征的体素滤波器
  pcl::VoxelGrid<PointType> surface_ds_filter_;

  /// @brief 优化参数 [roll, pitch, yaw, x, y, z]
  float transform_to_be_mapped_[6];
  /// @brief 优化参数转化为变换矩阵
  Eigen::Affine3f trans_point_associate_to_map_;

  /// @brief corner原始特征点保存序列(OMP优化)
  std::vector<PointType> corner_feature_original_vec_;
  /// @brief corner点优化参数序列(OMP优化)
  std::vector<PointType> corner_feature_coeff_vec_;
  /// @brief corner原始特征点标志位(OMP优化)
  std::vector<bool> corner_feature_origianl_flag_vec_;
  /// @brief surface原始特征点保存序列(OMP优化)
  std::vector<PointType> surface_feature_original_vec_;
  /// @brief surface点优化参数序列(OMP优化)
  std::vector<PointType> surface_feature_coeff_vec_;
  /// @brief surface原始特征点标志位(OMP优化)
  std::vector<bool> surface_feature_origianl_flag_vec_;

  /// @brief 原始特征点
  CloudTypePtr feature_cloud_original_;
  /// @brief 优化参数集合
  CloudTypePtr corff_sel_;

  /// @brief FastLOAM优化对象(姿态)
  double pose_parameters_[7];
  /// @brief 映射 旋转
  Eigen::Map<Eigen::Quaterniond> opt_pose_q_;
  /// @brief 映射 平移
  Eigen::Map<Eigen::Vector3d> opt_pose_t_;
  /// @brief corner关联保存序列(OMP优化)
  std::vector<CornerAssociation> corner_feature_association_vec_;
  /// @brief surface关联保存序列(OMP优化)
  std::vector<SurfaceAssociation> surface_feature_association_vec_;
  /// @brief 最新的优化旋转(用于检测收敛)
  Eigen::Quaterniond latest_opt_pose_q_;
  /// @brief 最新的优化平移(用于检测收敛)
  Eigen::Vector3d latest_opt_pose_t_;
};

}  // namespace multi_sensor_mapping

#endif
