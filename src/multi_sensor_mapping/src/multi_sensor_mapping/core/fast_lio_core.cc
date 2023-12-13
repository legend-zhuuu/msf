#include "multi_sensor_mapping/core/fast_lio_core.h"

#include "ikd-Tree/ikd_Tree_impl.h"
#include "multi_sensor_mapping/frontend/imu/imu_processor.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/utils/utils_log.h"

#define SKEW_SYM_MATRIX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]

template class KD_TREE<PointType>;

namespace multi_sensor_mapping {

FastLIOCore::FastLIOCore()
    : local_map_initialized_(false),
      efk_initialized_flag_(false),
      kdtree_delete_counter_(0) {}

void FastLIOCore::SetParams(
    const std::shared_ptr<ExtrinsicParams>& _extrinsic_params,
    const std::shared_ptr<FastLIOParams>& _lio_params) {
  extrinsic_params_ = _extrinsic_params;
  lio_params_ = _lio_params;

  Eigen::Matrix4d mat_lidar_to_imu =
      extrinsic_params_->imu_to_baselink.inverse() *
      extrinsic_params_->lidar_to_baselink[LidarName::LIDAR_A];
  lidar_to_imu_translation_ = mat_lidar_to_imu.block<3, 1>(0, 3);
  lidar_to_imu_rotation_ =
      Eigen::Quaterniond(mat_lidar_to_imu.block<3, 3>(0, 0));

  cube_length_ = _lio_params->cube_length;
  delete_range_ = _lio_params->delete_range;
}

void FastLIOCore::InitializeAll() {
  // 计算外参
  Eigen::Matrix4d mat_lidar_to_imu =
      extrinsic_params_->imu_to_baselink.inverse() *
      extrinsic_params_->lidar_to_baselink[LidarName::LIDAR_A];

  // 初始化IMU处理器
  imu_processor_ = std::make_shared<ImuProcessor>();
  imu_processor_->SetGyrCov(Eigen::Vector3d(
      lio_params_->gyro_cov, lio_params_->gyro_cov, lio_params_->gyro_cov));
  imu_processor_->SetAccCov(Eigen::Vector3d(
      lio_params_->accel_cov, lio_params_->accel_cov, lio_params_->accel_cov));
  imu_processor_->SetGyrBiasCov(Eigen::Vector3d(lio_params_->gyro_bias_cov,
                                                lio_params_->gyro_bias_cov,
                                                lio_params_->gyro_bias_cov));
  imu_processor_->SetAccBiasCov(Eigen::Vector3d(lio_params_->accel_bias_cov,
                                                lio_params_->accel_bias_cov,
                                                lio_params_->accel_bias_cov));
  imu_processor_->SetExtrinsic(lidar_to_imu_translation_,
                               lidar_to_imu_rotation_.toRotationMatrix());

  feature_undistort_ = CloudTypePtr(new CloudType);
  feature_undistort_downsampled_body_ = CloudTypePtr(new CloudType);
  feature_undistort_downsampled_world_ = CloudTypePtr(new CloudType);
  norm_vec_ = CloudTypePtr(new CloudType);
  laser_cloud_origin_ = CloudTypePtr(new CloudType(10000, 1));
  corr_norm_vec_ = CloudTypePtr(new CloudType(10000, 1));

  feature_voxel_filter_.setLeafSize(lio_params_->feature_ds_leaf_size,
                                    lio_params_->feature_ds_leaf_size,
                                    lio_params_->feature_ds_leaf_size);

  point_selected_surf_flag_.resize(10000, true);
  res_last_.resize(10000, -1000.0);

  // 初始化KF
  std::vector<double> epsi(23, 0.001);
  kf_.init_dyn_share(
      get_f, df_dx, df_dw,
      [this](state_ikfom& s,
             esekfom::dyn_share_datastruct<double>& ekfom_data) {
        ObsModel(s, ekfom_data);
      },
      lio_params_->max_iterations, epsi.data());
}

void FastLIOCore::OdometryOnceScan(const TimedCloudData& _cloud_data,
                                   const std::deque<IMUData>& _imu_data_list) {
  /// IMU推算+去畸变
  imu_processor_->Process(_cloud_data, _imu_data_list, kf_, feature_undistort_);
  latest_state_ = kf_.get_x();

  lidar_position_ =
      latest_state_.pos + latest_state_.rot * lidar_to_imu_translation_;
  lidar_rotation_ = latest_state_.rot * lidar_to_imu_rotation_;
  current_cloud_timestamp_ =
      _cloud_data.timestamp + _cloud_data.cloud->points.back().time;

  if (feature_undistort_->empty()) {
    AWARN_F("[FastLIOCore] Feature Undistorted is empty, skip this scan");
    return;
  }

  efk_initialized_flag_ = true;

  // 动态移除地图点
  LaserMapFovSegment();

  // 特征点降采样
  feature_voxel_filter_.setInputCloud(feature_undistort_);
  feature_voxel_filter_.filter(*feature_undistort_downsampled_body_);

  feature_ds_size_ = feature_undistort_downsampled_body_->size();
  // ikd tree 初始化
  if (ikd_tree_.Root_Node == nullptr) {
    if (feature_ds_size_ > 5) {
      ikd_tree_.set_downsample_param(lio_params_->map_ds_leaf_size);
      feature_undistort_downsampled_world_->resize(feature_ds_size_);
      for (int i = 0; i < feature_ds_size_; i++) {
        PointBodyToWorld(&(feature_undistort_downsampled_body_->points[i]),
                         &(feature_undistort_downsampled_world_->points[i]));
      }
      ikd_tree_.Build(feature_undistort_downsampled_world_->points);
    }

    return;
  }

  int feature_from_map_num = ikd_tree_.validnum();

  // ICP 匹配 以及 IKF更新
  if (feature_ds_size_ < 5) {
    AWARN_F(
        "[FastLIOCore] Feature Undistorted is too low, skip this "
        "scan");
    return;
  }

  norm_vec_->resize(feature_ds_size_);
  feature_undistort_downsampled_world_->resize(feature_ds_size_);

  nearest_points_.resize(feature_ds_size_);
  latest_state_ = kf_.get_x();

  double solve_H_time = 0;
  kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
  latest_state_ = kf_.get_x();
  lidar_position_ =
      latest_state_.pos + latest_state_.rot * lidar_to_imu_translation_;
  lidar_rotation_ = latest_state_.rot * lidar_to_imu_rotation_;

  // 添加点到KD tree中
  MapIcremental();
}

PoseData FastLIOCore::GetCurrentPose() {
  PoseData pose_data;
  pose_data.timestamp = current_cloud_timestamp_;
  pose_data.position = lidar_position_;
  pose_data.orientation = lidar_rotation_;
  return pose_data;
}

CloudTypePtr FastLIOCore::GetCurrentCloud() { return feature_undistort_; }

void FastLIOCore::ObsModel(state_ikfom& s,
                           esekfom::dyn_share_datastruct<double>& ekfom_data) {
  float total_residual = 0.0;
  for (int i = 0; i < feature_ds_size_; i++) {
    PointType& point_body = feature_undistort_downsampled_body_->points[i];
    PointType& point_world = feature_undistort_downsampled_world_->points[i];

    Eigen::Vector3d p_body(point_body.x, point_body.y, point_body.z);
    Eigen::Vector3d p_global(
        s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    std::vector<float> point_search_sq_dis(NUM_MATCH_POINTS);

    auto& points_near = nearest_points_[i];

    if (ekfom_data.converge) {
      /** Find the closest surfaces in the map **/
      ikd_tree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                               point_search_sq_dis);
      point_selected_surf_flag_[i] =
          points_near.size() < NUM_MATCH_POINTS
              ? false
              : point_search_sq_dis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
    }

    if (!point_selected_surf_flag_[i]) continue;

    Eigen::Vector4f plane_abcd;
    point_selected_surf_flag_[i] = false;

    if (EstimatePlane(plane_abcd, points_near, 0.1f)) {
      float pd2 = plane_abcd(0) * point_world.x +
                  plane_abcd(1) * point_world.y +
                  plane_abcd(2) * point_world.z + plane_abcd(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s > 0.9) {
        point_selected_surf_flag_[i] = true;
        norm_vec_->points[i].x = plane_abcd(0);
        norm_vec_->points[i].y = plane_abcd(1);
        norm_vec_->points[i].z = plane_abcd(2);
        norm_vec_->points[i].intensity = pd2;
        res_last_[i] = abs(pd2);
      }
    }
  }

  int effct_feat_num = 0;
  for (int i = 0; i < feature_ds_size_; i++) {
    if (point_selected_surf_flag_[i]) {
      laser_cloud_origin_->points[effct_feat_num] =
          feature_undistort_downsampled_body_->points[i];
      corr_norm_vec_->points[effct_feat_num] = norm_vec_->points[i];
      total_residual += res_last_[i];
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    AWARN_F("No Effective Points!");
    return;
  }

  ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);  // 23
  ekfom_data.h.resize(effct_feat_num);

  for (int i = 0; i < effct_feat_num; i++) {
    const PointType& laser_p = laser_cloud_origin_->points[i];
    Eigen::Vector3d point_this_be(laser_p.x, laser_p.y, laser_p.z);
    Eigen::Matrix3d point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRIX(point_this_be);
    Eigen::Vector3d point_this =
        s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    Eigen::Matrix3d point_crossmat;
    point_crossmat << SKEW_SYM_MATRIX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType& norm_p = corr_norm_vec_->points[i];
    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    Eigen::Vector3d C(s.rot.conjugate() * norm_vec);
    Eigen::Vector3d A(point_crossmat * C);

    ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
        VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;
  }
}

void FastLIOCore::LaserMapFovSegment() {
  cube_need_rm_.clear();
  kdtree_delete_counter_ = 0;

  if (!local_map_initialized_) {
    for (int i = 0; i < 3; i++) {
      local_map_points_.vertex_min[i] = lidar_position_(i) - cube_length_ / 2.0;
      local_map_points_.vertex_max[i] = lidar_position_(i) + cube_length_ / 2.0;
    }
    local_map_initialized_ = true;
    return;
  }

  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] =
        fabs(lidar_position_(i) - local_map_points_.vertex_min[i]);
    dist_to_map_edge[i][1] =
        fabs(lidar_position_(i) - local_map_points_.vertex_max[i]);
    //如果距离边缘过近 则需要进行地图的挪动
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * delete_range_ ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * delete_range_)
      need_move = true;
  }

  if (!need_move) return;

  BoxPointType new_local_map_points, tmp_box_points;
  new_local_map_points = local_map_points_;
  float mov_dist =
      std::max((cube_length_ - 2.0 * MOV_THRESHOLD * delete_range_) * 0.5 * 0.9,
               double(delete_range_ * (MOV_THRESHOLD - 1)));

  for (int i = 0; i < 3; i++) {
    tmp_box_points = local_map_points_;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * delete_range_) {
      new_local_map_points.vertex_max[i] -= mov_dist;
      new_local_map_points.vertex_min[i] -= mov_dist;
      tmp_box_points.vertex_min[i] =
          new_local_map_points.vertex_max[i] - mov_dist;
      cube_need_rm_.push_back(tmp_box_points);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * delete_range_) {
      new_local_map_points.vertex_max[i] += mov_dist;
      new_local_map_points.vertex_min[i] += mov_dist;
      tmp_box_points.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
      cube_need_rm_.push_back(tmp_box_points);
    }
  }

  local_map_points_ = new_local_map_points;

  PointVector points_history;
  ikd_tree_.acquire_removed_points(points_history);

  if (cube_need_rm_.size() > 0)
    kdtree_delete_counter_ = ikd_tree_.Delete_Point_Boxes(cube_need_rm_);
}

void FastLIOCore::PointBodyToWorld(PointType const* const pi,
                                   PointType* const po) {
  Eigen::Vector3d p_body(pi->x, pi->y, pi->z);
  Eigen::Vector3d p_global(
      latest_state_.rot *
          (latest_state_.offset_R_L_I * p_body + latest_state_.offset_T_L_I) +
      latest_state_.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void FastLIOCore::MapIcremental() {
  PointVector point_to_add;
  PointVector point_no_need_downsample;
  point_to_add.reserve(feature_ds_size_);
  point_no_need_downsample.reserve(feature_ds_size_);

  for (int i = 0; i < feature_ds_size_; i++) {
    PointBodyToWorld(&(feature_undistort_downsampled_body_->points[i]),
                     &(feature_undistort_downsampled_world_->points[i]));

    if (!nearest_points_[i].empty() && efk_initialized_flag_) {
      const PointVector& points_near = nearest_points_[i];
      bool need_add = true;
      BoxPointType Box_of_Point;
      PointType downsample_result, mid_point;
      mid_point.x = floor(feature_undistort_downsampled_world_->points[i].x /
                          lio_params_->map_ds_leaf_size) *
                        lio_params_->map_ds_leaf_size +
                    0.5 * lio_params_->map_ds_leaf_size;
      mid_point.y = floor(feature_undistort_downsampled_world_->points[i].y /
                          lio_params_->map_ds_leaf_size) *
                        lio_params_->map_ds_leaf_size +
                    0.5 * lio_params_->map_ds_leaf_size;
      mid_point.z = floor(feature_undistort_downsampled_world_->points[i].z /
                          lio_params_->map_ds_leaf_size) *
                        lio_params_->map_ds_leaf_size +
                    0.5 * lio_params_->map_ds_leaf_size;
      float dist = PointDistance(
          feature_undistort_downsampled_world_->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) >
              0.5 * lio_params_->map_ds_leaf_size &&
          fabs(points_near[0].y - mid_point.y) >
              0.5 * lio_params_->map_ds_leaf_size &&
          fabs(points_near[0].z - mid_point.z) >
              0.5 * lio_params_->map_ds_leaf_size) {
        point_no_need_downsample.push_back(
            feature_undistort_downsampled_world_->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        if (PointDistance(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        point_to_add.push_back(feature_undistort_downsampled_world_->points[i]);
      }
    } else {
      point_to_add.push_back(feature_undistort_downsampled_world_->points[i]);
    }
  }

  int add_point_size = ikd_tree_.Add_Points(point_to_add, true);
  ikd_tree_.Add_Points(point_no_need_downsample, false);
  add_point_size = point_to_add.size() + point_no_need_downsample.size();
}

bool FastLIOCore::EstimatePlane(Eigen::Vector4f& _pca_result,
                                const PointVector& _point,
                                const float& _threshold) {
  Eigen::Matrix<float, NUM_MATCH_POINTS, 3> A;
  Eigen::Matrix<float, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = _point[j].x;
    A(j, 1) = _point[j].y;
    A(j, 2) = _point[j].z;
  }

  Eigen::Matrix<float, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

  float n = normvec.norm();
  _pca_result(0) = normvec(0) / n;
  _pca_result(1) = normvec(1) / n;
  _pca_result(2) = normvec(2) / n;
  _pca_result(3) = 1.0 / n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (fabs(_pca_result(0) * _point[j].x + _pca_result(1) * _point[j].y +
             _pca_result(2) * _point[j].z + _pca_result(3)) > _threshold) {
      return false;
    }
  }
  return true;
}

}  // namespace multi_sensor_mapping