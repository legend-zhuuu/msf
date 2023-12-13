
#ifndef MSM_NDT_SCAN_MATCHER_H
#define MSM_NDT_SCAN_MATCHER_H

#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "pclomp/ndt_omp.h"

namespace multi_sensor_mapping {

/**
 * @brief The ScanMatcher 基于NDT的点云配准器
 */
class NdtScanMatcher {
 public:
  typedef std::shared_ptr<NdtScanMatcher> Ptr;

  /**
   * @brief NdtScanMatcher 构造函数
   */
  NdtScanMatcher();

  /**
   * @brief NdtScanMatcher 构造函数
   * @param _target_resolution
   * @param _source_resolution
   * @param _num_cores
   */
  NdtScanMatcher(double _target_resolution, double _source_resolution,
                 int _num_cores);

  /**
   * @brief 设置目标点云
   *
   * @param _target_cloud
   */
  void SetTargetCloud(CloudTypePtr& _target_cloud);

  /**
   * @brief Match 点云配准(核心函数)
   * @param _source_cloud
   * @param _initial_pose
   * @param _pose_estimate
   * @return
   */
  bool Match(CloudTypePtr _source_cloud, const Eigen::Matrix4f& _initial_pose,
             Eigen::Matrix4f& _pose_estimate);

  /**
   * @brief GetMatchScore 获取配准分数
   * @return
   */
  double GetMatchScore();

 private:
  /**
   * @brief InitMatcher 根据配准方式，初始化配准模块
   */
  void InitMatcher();

  /**
   * @brief 点云降采样
   *
   * @param _input_cloud
   * @param _output_cloud
   */
  void DownSampleCloud(CloudTypePtr _input_cloud, CloudTypePtr _output_cloud);

 private:
  /// @brief ndt栅格大小
  double target_resolution_;
  /// @brief source点云降采样参数
  double source_resolution_;
  /// @brief NDT配准多线程数量
  int num_cores_;
  /// @brief 配准分数
  double match_score_;

  /// @brief 栅格滤波
  pcl::VoxelGrid<PointType> cloud_filter_;
  /// @brief NDT指针
  std::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>>
      ndt_ptr_;
};

}  // namespace multi_sensor_mapping

#endif  // PROJECT_SCANMATCHER_H
