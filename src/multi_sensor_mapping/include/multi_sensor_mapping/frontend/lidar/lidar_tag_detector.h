#ifndef MSM_LIDAR_TAG_DETECTOR_H
#define MSM_LIDAR_TAG_DETECTOR_H

#include <memory>

#include "multi_sensor_mapping/utils/sensor_data.h"

namespace multi_sensor_mapping {

/**
 * @brief ROI区域
 *
 */
struct ROIArea {
  /// @brief 最小边界
  Eigen::Vector3f min_boundary;
  /// @brief 最大边界
  Eigen::Vector3f max_boundary;

  CloudTypePtr inner_cloud;

  Eigen::Vector4f plane_coeff;
};

struct TagCluster {
  /// @brief 检验通过
  bool valid = true;

  /// @brief 表征tag的五个点，上下左右中
  std::vector<Eigen::Vector4d> vertex_points;
  /// @brief 平面法向量
  Eigen::Vector4d plane_normal_vec;
  /// @brief tag内点 ceres-solver 用
  std::vector<Eigen::Vector3d> inner_points;
  /// @brief 有序点元 从下至上 从左至右(-x轴为奇异处)
  std::vector<CloudType> ordered_cloud;
  /// @brief 边界点
  std::vector<CloudType> edge_cloud;

  // Eigen::Vector3f center_point;
  struct CenterPoint {
    Eigen::Vector3f point;
    float angle;
  } center_point;
};

/// @brief angle in [0,2*MPI]
/// @param _point
/// @return
float GetAngle(const Eigen::Vector3f& _point);

/**
 * @brief 激光标签检测器
 *
 */
class LidarTagDetector {
 public:
  /**
   * @brief Construct a new Lidar Tag Detector object
   *
   */
  LidarTagDetector();

  /**
   * @brief Construct a new Lidar Tag Detector object
   *
   * @param _num_points_plane
   * @param _intensity_threshold
   * @param _depth_threshold
   * @param _tag_size
   * @param _leaf_size
   * @param _layer_size
   */
  LidarTagDetector(int _num_points_plane, float _intensity_threshold,
                   float _depth_threshold, float _tag_size, float _leaf_size,
                   int _layer_size);

  /**
   * @brief 检测标签
   *
   * @param _input_cloud
   * @return int
   */
  int DetectTag(const VelRTPointCloudPtr& _input_cloud);

  /**
   * @brief Get the ROIArea object 获取ROI区域
   *
   * @return std::vector<ROIArea>
   */
  std::vector<ROIArea> GetROIArea();

  /**
   * @brief Get the Edgecloud object 获取边缘点云
   *
   * @return CloudTypePtr
   */
  CloudTypePtr GetEdgecloud() { return edge_cloud_; }

  /**
   * @brief Get the Filtered Cloud object 获取过滤点云
   *
   * @return CloudTypePtr
   */
  CloudTypePtr GetFilteredCloud() { return cloud_filtered_; }

  /**
   * @brief Get the Tag Pose object
   *
   * @return std::vector<PoseData>
   */
  std::vector<PoseData> GetTagPose() { return tag_pose_; }

 private:
  /**
   * @brief 转化为有序点云
   *
   * @param _input_cloud
   * @param _ordered_cloud
   */
  void FillInOrderedCloud(const VelRTPointCloudPtr& _input_cloud,
                          std::vector<std::vector<VelRTPoint>>& _ordered_cloud);

  /**
   * @brief 不依赖线束信息 支持重复性和非重复性扫描激光雷达
   *
   * @param _input_cloud    无线束信息的点云
   * @param _ordered_cloud  分层有序点云
   */
  bool CreateTagCluster(const CloudTypePtr& _input_cloud,
                        TagCluster& _tag_cluster);

  /**
   * @brief 根据深度提取边缘
   *
   * @param _ordered_cloud
   * @param _edge_cloud
   */
  void GradientAndGroupEdgs(const std::vector<CloudType>& _ordered_cloud,
                            std::vector<CloudType>& _edge_cloud,
                            TagCluster& _tag_cluster);

  /**
   * @brief Get the Edge Points object
   *
   * @param _layer_cloud  单层点云
   * @param _edge_point   两侧点云
   */
  int GetEdgePoints(const CloudType& _layer_cloud,
                    std::vector<PointType>& _edge_point);

  /**
   * @brief Get the Vertex Points object from the Edge line
   *
   * @param _line_side      four edge line from lidar tag
   * @param _vertex_points  four vertex points and center point,Zero() for
   * invalid point
   * @return valid point number
   */
  int GetVertexPoints(const std::vector<CloudType>& _line_side,
                      std::vector<Eigen::Vector4d>& _vertex_points);

  /**
   * @brief 提取平面
   *
   * @param _cloud
   * @param _coeffs
   * @param _cloud_inliers
   * @return true
   * @return false
   */
  bool EstimatePlane(const CloudTypePtr _cloud, Eigen::Vector4f& _coeffs,
                     CloudTypePtr& _cloud_inliers);

  /**
   * @brief 使用 ceres-solver 优化求解变换关系
   *
   * @param _vertex_tag
   * @param _init_transform
   * @param _optimized_transform_
   * @return true
   * @return false
   */
  bool TransformSolver(const std::vector<Eigen::Vector3d>& _inner_points,
                       const Eigen::Matrix4d& _init_transform,
                       Eigen::Matrix4d& _optimized_transform_);

 private:
  /// @brief 每个平面的点数
  int num_points_for_plane_;

  /// @brief 强度阈值
  float intensity_threshold_;
  /// @brief 深度阈值
  float depth_threshold_;
  /// @brief tag的长宽
  float tag_size_;
  /// @brief 体素大小
  float leaf_size_;
  /// @brief 单层高度
  int layer_size_;

  /// @brief ROI区域点云
  CloudTypePtr roi_cloud_;
  /// @brief 边缘区域点云
  CloudTypePtr edge_cloud_;

  /// @brief ROI区域
  std::vector<ROIArea> roi_areas_;
  /// @brief 可能的 tag 点云聚类簇群
  std::vector<CloudTypePtr> possible_tag_clusters_;
  /// @brief tag姿态
  std::vector<PoseData> tag_pose_;

  CloudTypePtr cloud_filtered_;
};

}  // namespace multi_sensor_mapping

#endif