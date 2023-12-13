#ifndef MSM_GICP_SCAN_MATCHER_H
#define MSM_GICP_SCAN_MATCHER_H

#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "pclomp/gicp_omp.h"

namespace multi_sensor_mapping {

class GICPScanMatcher {
 public:
  typedef std::shared_ptr<GICPScanMatcher> Ptr;

  /**
   * @brief Construct a new Gicp Scan Matcher object 构造函数
   *
   */
  GICPScanMatcher();

  /**
   * @brief Construct a new GICPScanMatcher object 构造函数
   *
   * @param _resolution
   */
  GICPScanMatcher(double _resolution);

  /**
   * @brief 设置目标点云
   *
   * @param _target_cloud
   */
  void SetTargetCloud(const CloudTypePtr& _target_cloud);

  /**
   * @brief Match 点云配准(核心函数)
   * @param _source_cloud
   * @param _initial_pose
   * @param _pose_estimate
   * @return
   */
  bool Match(const CloudTypePtr& _source_cloud,
             const Eigen::Matrix4f& _initial_pose,
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
  /// @brief 分辨率
  double resolution_;
  /// @brief 配准分数
  double match_score_;

  /// @brief 栅格滤波
  pcl::VoxelGrid<PointType> cloud_filter_;
  /// @brief GICP 指针
  std::shared_ptr<
      pclomp::GeneralizedIterativeClosestPoint<PointType, PointType>>
      gicp_ptr_;
};
}  // namespace multi_sensor_mapping

#endif