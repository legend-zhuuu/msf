#include "multi_sensor_mapping/frontend/lidar/lidar_tag_detector.h"

#include <pcl/common/intersections.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "multi_sensor_mapping/factor/auto_diff/lidar_tag_factor.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

float GetAngle(const Eigen::Vector3f& _point) {
  float angle = acos(_point(0) /
                     std::sqrt(_point(0) * _point(0) + _point(1) * _point(1)));

  if (_point(1) < 0) {
    angle = -angle;
  }

  return angle;
}

static bool cmp_angle(const PointType& p_a, const PointType& p_b) {
  float angle_a = GetAngle(p_a.getVector3fMap());
  float angle_b = GetAngle(p_b.getVector3fMap());

  return angle_a > angle_b;
}

const double sqrt_2 = 1.41421356237309;

LidarTagDetector::LidarTagDetector()
    : num_points_for_plane_(3),
      intensity_threshold_(254),
      depth_threshold_(0.4f),
      tag_size_(0.6f),
      leaf_size_(0.02f),
      layer_size_(4),
      roi_cloud_(new CloudType),
      edge_cloud_(new CloudType),
      cloud_filtered_(new CloudType) {}

LidarTagDetector::LidarTagDetector(int _num_points_plane,
                                   float _intensity_threshold,
                                   float _depth_threshold, float _tag_size,
                                   float _leaf_size, int _layer_size)
    : num_points_for_plane_(_num_points_plane),
      intensity_threshold_(_intensity_threshold),
      depth_threshold_(_depth_threshold),
      tag_size_(_tag_size),
      leaf_size_(_leaf_size),
      layer_size_(_layer_size),
      roi_cloud_(new CloudType),
      edge_cloud_(new CloudType),
      cloud_filtered_(new CloudType) {}

int LidarTagDetector::DetectTag(const VelRTPointCloudPtr& _input_cloud) {
  possible_tag_clusters_.clear();
  roi_cloud_->clear();
  roi_areas_.clear();
  tag_pose_.clear();

  /// Step1  提取高强度点云
  for (size_t i = 0; i < _input_cloud->size(); i++) {
    if (_input_cloud->points[i].intensity > intensity_threshold_) {
      PointType p;
      p.x = _input_cloud->points[i].x;
      p.y = _input_cloud->points[i].y;
      p.z = _input_cloud->points[i].z;
      p.intensity = _input_cloud->points[i].intensity;
      roi_cloud_->push_back(p);
    }
  }

  if (roi_cloud_->size() < 20) {
    // AINFO_F("[LidarTagDetector] ROI cloud size is less than 20");
    return 0;
  }

  /// Step 2 点云聚类
  pcl::search::KdTree<PointType>::Ptr tree_raw_cloud(
      new pcl::search::KdTree<PointType>);
  tree_raw_cloud->setInputCloud(roi_cloud_);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(0.2);
  ec.setMinClusterSize(20);    // 最小聚类数
  ec.setMaxClusterSize(1000);  // 最大聚类数
  ec.setSearchMethod(tree_raw_cloud);
  ec.setInputCloud(roi_cloud_);
  ec.extract(cluster_indices);

  /// Step3 提取反光 tag中心
  for (pcl::PointIndices cluster : cluster_indices) {
    TagCluster tag_cluster;
    Eigen::Vector3f center(0, 0, 0);
    int point_cnt = 0;

    for (const int& idx : cluster.indices) {
      center += roi_cloud_->points[idx].getVector3fMap();
      point_cnt++;
    }

    center = center / point_cnt;
    tag_cluster.center_point.point = center;
    tag_cluster.center_point.angle = GetAngle(center);

    // 计算ROI区域
    double extern_distance = tag_size_ * sqrt_2 / 2.0;
    Eigen::Vector3f min_points =
        center -
        Eigen::Vector3f(extern_distance, extern_distance, extern_distance);
    Eigen::Vector3f max_points =
        center +
        Eigen::Vector3f(extern_distance, extern_distance, extern_distance);

    // 遍历点云,并进行平面提取
    CloudTypePtr possible_tag_cluster(new CloudType);
    // VelRTPointCloudPtr cloud_filtered(new VelRTPointCloud());
    for (size_t i = 0; i < _input_cloud->size(); i++) {
      auto raw_point = _input_cloud->points[i];
      if (raw_point.x < max_points[0] && raw_point.x > min_points[0] &&
          raw_point.y < max_points[1] && raw_point.y > min_points[1] &&
          raw_point.z < max_points[2] && raw_point.z > min_points[2]) {
        PointType p;
        VelRTPoint p_vel;
        p.x = p_vel.x = raw_point.x;
        p.y = p_vel.y = raw_point.y;
        p.z = p_vel.z = raw_point.z;
        p.intensity = p_vel.intensity = raw_point.intensity;
        p_vel.ring = raw_point.ring;
        possible_tag_cluster->push_back(p);
        // cloud_filtered->push_back(p_vel);
      }
    }

    if (possible_tag_cluster->size() > 40) {
      possible_tag_clusters_.push_back(possible_tag_cluster);
    } else {
      AINFO_F(
          "[LidarTagDetector] Possible tag cloud cluster size is less than "
          "40");
    }

    // 降采样
    CloudTypePtr cloud_filtered(new CloudType());
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(possible_tag_cluster);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud_filtered);

    if (cloud_filtered->size() < 200) {
      AINFO_F("[LidarTagDetector] Filtered cloud size is less than 200");
      continue;
    }

    // 平面提取
    CloudTypePtr cloud_inliers(new CloudType);
    Eigen::Vector4f plane_coeffs;
    EstimatePlane(cloud_filtered, plane_coeffs, cloud_inliers);
    tag_cluster.plane_normal_vec = plane_coeffs.cast<double>();

    // 校验并获取tag相关信息
    CreateTagCluster(cloud_inliers, tag_cluster);

    if (!tag_cluster.valid) {
      continue;
    }

    // 位姿估计
    Eigen::Vector3d vector_z = tag_cluster.plane_normal_vec.head(3);
    if (tag_cluster.plane_normal_vec(3) < 0) {
      vector_z *= (-1);
    }
    Eigen::Vector3d vector_x = tag_cluster.vertex_points[2].head(3) -
                               tag_cluster.vertex_points[4].head(3);

    Eigen::Quaterniond rotation = Eigen::Quaterniond().setFromTwoVectors(
        Eigen::Vector3d::UnitZ(), vector_z);

    Eigen::Vector3d transformed_vec_x = rotation.inverse() * vector_x;
    // std::cout << transformed_vec_x.transpose() << std::endl;

    Eigen::Quaterniond rotation_x = Eigen::Quaterniond().setFromTwoVectors(
        Eigen::Vector3d::UnitX(), transformed_vec_x);

    double yaw, pitch, roll;
    utils::RoationMatrixd2YPR(rotation_x.toRotationMatrix(), yaw, pitch, roll);
    // std::cout << "roll : " << roll / M_PI * 180.0 << std::endl;
    // std::cout << "pitch : " << pitch / M_PI * 180.0 << std::endl;
    // std::cout << "yaw : " << yaw / M_PI * 180.0 << std::endl;

    Eigen::Quaterniond rotation_x_refined =
        Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1)));

    Eigen::Vector3d translation = tag_cluster.vertex_points[4].head(3);

    PoseData pose;
    pose.position = translation;
    pose.orientation = rotation * rotation_x_refined;
    tag_pose_.push_back(pose);

    // Eigen::Matrix4d init_transform = Eigen::Matrix4d::Identity();
    // init_transform.block<3, 3>(0, 0) = rotation;
    // init_transform.block<3, 1>(0, 3) = translation;

    // Eigen::Matrix4d optimized_transform = Eigen::Matrix4d::Identity();
    // TransformSolver(tag_cluster.inner_points, init_transform,
    //                 optimized_transform);

    // 可视化
    cloud_filtered_->clear();
    for (std::size_t i = 0; i < tag_cluster.ordered_cloud.size(); i++) {
      (*cloud_filtered_) += tag_cluster.ordered_cloud[i];
    }

    edge_cloud_->clear();
    for (std::size_t i = 0; i < tag_cluster.edge_cloud.size(); i++) {
      (*edge_cloud_) += tag_cluster.edge_cloud[i];
    }
    for (std::size_t i = 0; i < tag_cluster.vertex_points.size(); i++) {
      CloudType vertex_cloud;
      Eigen::Vector4d vertex_point = tag_cluster.vertex_points[i];
      if (vertex_point != Eigen::Vector4d::Zero()) {
        PointType p;
        p.x = vertex_point[0];
        p.y = vertex_point[1];
        p.z = vertex_point[2];
        p.intensity = 255;
        vertex_cloud.points.push_back(p);
      }
      (*edge_cloud_) += vertex_cloud;
    }

    if (cloud_inliers->size() < 20) {
      continue;
    }

    Eigen::Vector4f min_points_3d, max_points_3d;
    pcl::getMinMax3D(*cloud_inliers, min_points_3d, max_points_3d);

    ROIArea roi_area;
    roi_area.min_boundary = min_points_3d.head(3);
    roi_area.max_boundary = max_points_3d.head(3);
    roi_area.inner_cloud = cloud_inliers;
    roi_area.plane_coeff = plane_coeffs;
    roi_areas_.push_back(roi_area);
  }

  return roi_areas_.size();
}

void LidarTagDetector::FillInOrderedCloud(
    const VelRTPointCloudPtr& _input_cloud,
    std::vector<std::vector<VelRTPoint>>& _ordered_cloud) {
  size_t points_num = _input_cloud->size();

  for (size_t i = 0; i < points_num; i++) {
    _ordered_cloud[_input_cloud->points[i].ring].push_back(
        _input_cloud->points[i]);
  }
}

bool LidarTagDetector::CreateTagCluster(const CloudTypePtr& _input_cloud,
                                        TagCluster& _tag_cluster) {
  // get set of z
  std::set<int> z_set;

  for (const auto& point : _input_cloud->points) {
    int z_100 = 0;
    if (point.z < 0) {
      z_100 = std::roundf(point.z * 100.0f / layer_size_) - 1;
    } else {
      z_100 = std::roundf(point.z * 100.0f / layer_size_) + 1;
    }

    if (z_set.find(z_100) == z_set.end()) {
      z_set.emplace(z_100);
    }
  }

  // 高度判定 45度数
  int tag_height = 0;
  auto iter_end = z_set.end();
  iter_end--;
  int end_index = *(iter_end);
  int start_index = *(z_set.begin());

  if (end_index * start_index < 0) {
    tag_height = (end_index - start_index - 1) * layer_size_;
  } else {
    tag_height = (end_index - start_index) * layer_size_;
  }

  if (tag_height / 100.0f < tag_size_ / 2.0f ||
      tag_height / 100.0f - 0.1f > tag_size_ * sqrt_2) {
    _tag_cluster.valid = false;
    AWARN_F("[LidarTagDetector] tag height:%f ", tag_height / 100.0f);
    return false;
  }

  // float max_angle = 0, min_angle = M_PI;
  // use z set divide cloud cluster to multi layers
  std::vector<CloudType> ordered_cloud(z_set.size());
  for (const auto& point : _input_cloud->points) {
    int z_100 = 0;
    if (point.z < 0) {
      z_100 = std::roundf(point.z * 100.0f / layer_size_) - 1;
    } else {
      z_100 = std::roundf(point.z * 100.0f / layer_size_) + 1;
    }
    int index =
        static_cast<int>(std::distance(z_set.begin(), z_set.find(z_100)));
    ordered_cloud[index].points.push_back(point);
    ordered_cloud[index].width++;
    if (!ordered_cloud[index].height) {
      ordered_cloud[index].height = 1;
    }
  }

  std::vector<CloudType> _edge_cloud;

  // 从中间点向外 寻找边界点
  for (size_t i = 0; i < z_set.size(); i++) {
    std::sort(ordered_cloud[i].points.begin(), ordered_cloud[i].points.end(),
              cmp_angle);
  }

  GradientAndGroupEdgs(ordered_cloud, _edge_cloud, _tag_cluster);

  return _tag_cluster.valid;
}

void LidarTagDetector::GradientAndGroupEdgs(
    const std::vector<CloudType>& _ordered_cloud,
    std::vector<CloudType>& _edge_cloud, TagCluster& _tag_cluster) {
  // 存储左右两侧的边界点
  std::vector<CloudType> edge_sides(2);
  float left_most_angle = -M_PI;
  int left_most_index = 0;
  float right_most_angle = M_PI;
  int right_most_index = 0;

  std::vector<Eigen::Vector3d> inner_points;

  for (std::size_t i = 0; i < _ordered_cloud.size(); i++) {
    std::vector<PointType> edge_point;
    int edge_type = GetEdgePoints(_ordered_cloud[i], edge_point);

    if (edge_type == 1) {
      edge_sides[0].points.push_back(edge_point[0]);

    } else if (edge_type == 2) {
      edge_sides[1].points.push_back(edge_point[0]);

    } else if (edge_type == 3) {
      edge_sides[0].points.push_back(edge_point[0]);
      edge_sides[1].points.push_back(edge_point[1]);
    }

    if (edge_type && i % 2 == 0) {
      for (std::size_t i = 0; i < edge_point.size(); i++) {
        inner_points.push_back(edge_point[i].getVector3fMap().cast<double>());
      }
      int center_index = _ordered_cloud[i].size() / 2;
      inner_points.push_back(_ordered_cloud[i]
                                 .points[center_index]
                                 .getArray3fMap()
                                 .cast<double>());
    }
  }

  // {右上，左上，左下，右下}
  std::vector<CloudType> line_sides(4);

  // 检测左侧边界点
  std::size_t left_points_num = edge_sides[0].points.size() - 1;
  int skip_flag_l = 0;
  for (size_t i = 0; i < left_points_num; i++) {
    if (skip_flag_l) {
      skip_flag_l = 0;
      continue;
    }
    Eigen::Vector3f point_a = edge_sides[0].points[i].getVector3fMap();
    Eigen::Vector3f point_b = edge_sides[0].points[i + 1].getVector3fMap();
    if (point_a == Eigen::Vector3f::Zero() ||
        point_b == Eigen::Vector3f::Zero()) {
      continue;
    }
    PointType new_point = edge_sides[0].points[i];

    if (std::fabs(point_a.z() - point_b.z()) < layer_size_ * 1.0f / 200.0f) {
      skip_flag_l = 1;
      float angle_a = GetAngle(point_a);
      float angle_b = GetAngle(point_b);

      // 较外侧为有效点 无效点则重置
      if (angle_a < angle_b) {
        new_point = edge_sides[0].points[i + 1];
        edge_sides[0].points[i] = PointType();
      } else {
        edge_sides[0].points[i + 1] = PointType();
      }
    }

    float angle = GetAngle(new_point.getVector3fMap());
    if (angle > left_most_angle) {
      left_most_angle = angle;
      left_most_index = i;
    }
  }

  // 将左侧边界点分割成上下两部分
  for (std::size_t i = 0; i < left_points_num + 1; i++) {
    PointType point = edge_sides[0].points[i];
    if (point.getVector3fMap() != Eigen::Vector3f::Zero()) {
      if (i < left_most_index) {
        // 左下
        line_sides[2].points.push_back(point);
      } else {
        // 左上
        line_sides[1].points.push_back(point);
      }
    }
  }

  // 检测右侧边界点
  std::size_t right_points_num = edge_sides[1].points.size() - 1;
  int skip_flag_r = 0;
  for (size_t i = 0; i < right_points_num; i++) {
    if (skip_flag_r) {
      skip_flag_r = 0;
      continue;
    }
    Eigen::Vector3f point_a = edge_sides[1].points[i].getVector3fMap();
    Eigen::Vector3f point_b = edge_sides[1].points[i + 1].getVector3fMap();
    if (point_a == Eigen::Vector3f::Zero() ||
        point_b == Eigen::Vector3f::Zero()) {
      continue;
    }
    PointType new_point = edge_sides[1].points[i];

    if (std::fabs(point_a.z() - point_b.z()) < layer_size_ * 1.0f / 200.0f) {
      skip_flag_r = 1;
      float angle_a = GetAngle(point_a);
      float angle_b = GetAngle(point_b);

      // 较外侧为有效点 无效点则重置
      if (angle_a > angle_b) {
        new_point = edge_sides[1].points[i + 1];
        edge_sides[1].points[i] = PointType();
      } else {
        edge_sides[1].points[i + 1] = PointType();
      }
    }

    float angle = GetAngle(new_point.getVector3fMap());
    if (angle < right_most_angle) {
      right_most_angle = angle;
      right_most_index = i;
    }
  }

  // 将右侧边界点分割成上下两部分
  for (std::size_t i = 0; i < right_points_num + 1; i++) {
    PointType point = edge_sides[1].points[i];
    if (point.getVector3fMap() != Eigen::Vector3f::Zero()) {
      if (i < right_most_index) {
        // 右下
        line_sides[3].points.push_back(point);
      } else {
        // 右上
        line_sides[0].points.push_back(point);
      }
    }
  }

  // get vertex points and center point
  std::vector<Eigen::Vector4d> vertex_points(5);
  int valid_num = GetVertexPoints(line_sides, vertex_points);

  if (valid_num == 5) {
    _tag_cluster.valid = true;
    _tag_cluster.vertex_points = vertex_points;
    _tag_cluster.inner_points = inner_points;
  } else {
    _tag_cluster.valid = false;
  }
}

int LidarTagDetector::GetVertexPoints(
    const std::vector<CloudType>& _line_side,
    std::vector<Eigen::Vector4d>& _vertex_points) {
  std::vector<pcl::ModelCoefficients> coefficients(4);
  std::vector<pcl::PointIndices> inliers(4);
  pcl::SACSegmentation<PointType> seg;

  // optinal
  seg.setOptimizeCoefficients(true);
  // mandatory
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.04);

  for (int i = 0; i < 4; i++) {
    seg.setInputCloud(_line_side[i].makeShared());
    seg.segment(inliers[i], coefficients[i]);
  }

  std::vector<Eigen::Vector4f> intersect_points(4);
  float dis_std = 0.02;

  for (int i = 0; i < 4; i++) {
    int next_index = (i + 1) % 4;
    pcl::lineWithLineIntersection(coefficients[i], coefficients[next_index],
                                  intersect_points[i], 0.04);
  }

  int valid_line = 0;
  std::vector<Eigen::Vector4d> vertex_points(5, Eigen::Vector4d::Identity());
  for (int i = 0; i < 4; i++) {
    int next_index = (i + 1) % 4;
    Eigen::Vector4f line = intersect_points[i] - intersect_points[next_index];
    if (fabs(line.norm() - tag_size_) < 2 * dis_std) {
      valid_line++;
      vertex_points[i] = intersect_points[i].cast<double>();
      vertex_points[i](3) = 1.0;
    }
  }

  if (valid_line == 4) {
    valid_line++;
    vertex_points[4] = (vertex_points[0] + vertex_points[1] + vertex_points[2] +
                        vertex_points[3]) /
                       4.0f;
  }

  _vertex_points = vertex_points;

  return valid_line;
}

int LidarTagDetector::GetEdgePoints(const CloudType& _layer_cloud,
                                    std::vector<PointType>& _edge_points) {
  int center_index = _layer_cloud.size() / 2;
  int edge_type = 0;

  int left_index = _layer_cloud.size();
  int right_index = 0;

  // 向左
  for (int i = center_index; i > 0; i--) {
    Eigen::Vector3f left_point = _layer_cloud.points[i - 1].getVector3fMap();
    Eigen::Vector3f mid_point = _layer_cloud.points[i].getVector3fMap();

    // 通过距离检测
    float near_dist = (left_point - mid_point).norm();
    if (near_dist > 0.1f || i == 1) {
      PointType p = _layer_cloud.points[i];
      _edge_points.push_back(p);
      if (i < left_index) {
        left_index = i;
      }
      break;
    }

    Eigen::Vector3f right_point = _layer_cloud.points[i + 1].getVector3fMap();
    // 向量差异
    float vector_diff = (left_point - 2 * mid_point + right_point).norm();

    if (vector_diff > 0.2f) {
      PointType p = _layer_cloud.points[i];
      _edge_points.push_back(p);
      if (i < left_index) {
        left_index = i;
      }
      break;
    }
  }

  if (_edge_points.size() == 1) {
    right_index = center_index;
    edge_type = 1;
  }

  //  向右
  for (int i = center_index; i < _layer_cloud.size(); i++) {
    Eigen::Vector3f right_point = _layer_cloud.points[i + 1].getVector3fMap();
    Eigen::Vector3f mid_point = _layer_cloud.points[i].getVector3fMap();

    // 通过距离检测
    float near_dist = (right_point - mid_point).norm();
    if (near_dist > 0.1f || i == _layer_cloud.size() - 2) {
      PointType p = _layer_cloud.points[i];
      _edge_points.push_back(p);
      if (i > right_index) {
        right_index = i;
      }
      break;
    }

    Eigen::Vector3f left_point = _layer_cloud.points[i - 1].getVector3fMap();
    // 向量差异
    float vector_diff = (left_point - 2 * mid_point + right_point).norm();

    if (vector_diff > 0.2f) {
      PointType p = _layer_cloud.points[i];
      _edge_points.push_back(p);
      if (i > right_index) {
        right_index = i;
      }
      break;
    }
  }

  if (!edge_type && _edge_points.size() == 1) {
    left_index = center_index;
    edge_type = 2;
  } else if (_edge_points.size() == 2) {
    edge_type = 3;
  }

  // // 保存内点
  // for (int i = left_index; i < right_index; i++) {
  //   _inner_points.push_back(
  //       _layer_cloud.points[i].getVector3fMap().cast<double>());
  // }

  return edge_type;
}

bool LidarTagDetector::EstimatePlane(const CloudTypePtr _cloud,
                                     Eigen::Vector4f& _coeffs,
                                     CloudTypePtr& _cloud_inliers) {
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;  /// Create the segmentation object
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);

  seg.setInputCloud(_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() < 20) {
    return false;
  }

  for (int i = 0; i < 4; i++) {
    _coeffs(i) = coefficients->values[i];
  }

  pcl::copyPointCloud<PointType>(*_cloud, *inliers, *_cloud_inliers);
  return true;
}

std::vector<ROIArea> LidarTagDetector::GetROIArea() { return roi_areas_; }

bool LidarTagDetector::TransformSolver(
    const std::vector<Eigen::Vector3d>& _inner_points,
    const Eigen::Matrix4d& _init_transform,
    Eigen::Matrix4d& _optimized_transform_) {
  Eigen::Quaterniond q(_init_transform.block<3, 3>(0, 0));
  q.normalize();
  Eigen::Vector3d translation(_init_transform.block<3, 1>(0, 3));

  // 取逆
  double transform_coeff[7] = {translation[0], translation[1], translation[2],
                               q.w(),          -q.x(),         -q.y(),
                               -q.z()};

  ceres::Problem problem;

  // set ceres options
  ceres::Solver::Summary summary;
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 10;
  options.num_threads = 1;
  options.minimizer_progress_to_stdout = true;

  // add residual blocks
  for (std::size_t i = 0; i < _inner_points.size(); i++) {
    ceres::CostFunction* cost_function =
        LidarTagFactor::Create(_inner_points[i], tag_size_ / 2.0);
    problem.AddResidualBlock(cost_function, nullptr, transform_coeff);
  }

  // optimization
  ceres::Solve(options, &problem, &summary);

  // update transform
  _optimized_transform_ = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond optimized_q(transform_coeff[3], transform_coeff[4],
                                 transform_coeff[5], transform_coeff[6]);
  _optimized_transform_.block<3, 3>(0, 0) = optimized_q.toRotationMatrix();
  _optimized_transform_.block<3, 1>(0, 3) = Eigen::Vector3d(
      transform_coeff[0], transform_coeff[1], transform_coeff[2]);

  return true;
}

}  // namespace multi_sensor_mapping