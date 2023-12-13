#include "multi_sensor_mapping/core/clins_core.h"

#include "ikd-Tree/ikd_Tree_impl.h"
#include "multi_sensor_mapping/map/lidar_map_session.h"
#include "multi_sensor_mapping/param/clins_params.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/spline/trajectory.h"
#include "multi_sensor_mapping/spline/trajectory_manager.h"
#include "multi_sensor_mapping/utils/utils_log.h"

template class KD_TREE<KRTPoint>;

namespace multi_sensor_mapping {

ClinsCore::ClinsCore()
    : param_initialized_flag_(false), keyframe_flag_(false) {}

void ClinsCore::SetParams(
    const std::shared_ptr<SensorParams>& _sensor_params,
    const std::shared_ptr<ExtrinsicParams>& _extrinsic_params,
    const std::shared_ptr<ClinsParams>& _mapper_params) {
  sensor_params_ = _sensor_params;
  extrinsic_params_ = _extrinsic_params;
  mapper_params_ = _mapper_params;

  keyframe_adding_time_ns_threshold_ =
      mapper_params_->keyframe_adding_time_threshold * 1e9;
  param_initialized_flag_ = true;
}

void ClinsCore::SetLidarMapSession(
    const std::shared_ptr<LidarMapSession>& _map_session) {
  active_lidar_map_session_ = _map_session;
}

void ClinsCore::InitializeAll() {
  if (!param_initialized_flag_) {
    AWARN_F("[ClinsCore] No input params !");
  }

  // 初始化轨迹
  trajectory_ = std::make_shared<Trajectory>(mapper_params_->knot_distance, 0);
  trajectory_->SetExtrinsicParams(sensor_params_, extrinsic_params_);

  // 初始化轨迹管理器
  trajectory_manager_ =
      std::make_shared<TrajectoryManager>(mapper_params_, trajectory_);
  feature_selected_flag_container_.resize(10000, true);
}

void ClinsCore::OdometryOneScan(TimedCloudData2& _cloud_data,
                                const std::vector<IMUData2>& _imu_data_list) {
  keyframe_flag_ = false;

  /// Step 1: IMU数据处理
  FeedImuDataVec(_imu_data_list);

  /// Step 2: 点云数据预处理
  if (trajectory_->GetDataStartTime() < 0) {
    AWARN_F("[ClinsCore] Trajectory data start time error !");
  } else {
    _cloud_data.scan_start_timestamp_ns -= trajectory_->GetDataStartTime();
    _cloud_data.scan_end_timestamp_ns -= trajectory_->GetDataStartTime();
  }
  PreprocessCloudData(_cloud_data);

  /// Step 3 : 拓展轨迹
  trajectory_manager_->PredictTrajectory(_cloud_data.scan_start_timestamp_ns,
                                         _cloud_data.scan_end_timestamp_ns);

  /// Step 4 : 激光去畸变
  KRTPointCloudPtr undistorted_feature(new KRTPointCloud);
  // 转换到全局坐标系
  trajectory_manager_->UndistortFeatures(_cloud_data, undistorted_feature,
                                         true);

  /// Step 5 : 数据关联
  // 初始化IKD tree
  if (ikd_tree_.Root_Node == nullptr) {
    if (undistorted_feature->size() > 5) {
      ikd_tree_.set_downsample_param(mapper_params_->feature_leaf_size);
      ikd_tree_.Build(undistorted_feature->points);
    }
    return;
  }

  nearest_points_.resize(undistorted_feature->size());
  int cor_cnt = FeatureAssociation(undistorted_feature);
  AINFO_F("[ClinsCore] Feature association count : %i ", cor_cnt);

  /// Step 6 : 轨迹优化
}

void ClinsCore::SetInitialState(const IMUState& _initial_state,
                                const IMUData2& _imu_data) {
  initial_state_ = _initial_state;
  trajectory_manager_->SetInitialState(initial_state_);

  trajectory_manager_->FeedImuData(_imu_data);
}

void ClinsCore::FeedImuDataVec(const std::vector<IMUData2>& _imu_data_list) {
  if (_imu_data_list.empty()) return;

  for (const auto& imu_data : _imu_data_list) {
    if (imu_data.gyro.norm() == 0 && imu_data.accel.norm() == 0) {
      AWARN_F("[ClinsCore] IMU gyro accel data error !");
    } else {
      trajectory_manager_->FeedImuData(imu_data);
    }
  }
}

void ClinsCore::PreprocessCloudData(const TimedCloudData2& _cloud_data) {
  current_feature_data_.scan_start_timestamp_ns =
      _cloud_data.scan_start_timestamp_ns;
  current_feature_data_.scan_end_timestamp_ns =
      _cloud_data.scan_end_timestamp_ns;

  int cloud_size = _cloud_data.cloud->size();
  if (!current_feature_data_.cloud) {
    current_feature_data_.cloud.reset(new KRTPointCloud);
  }
  current_feature_data_.cloud->reserve(cloud_size);

  // 降采样
  for (int i = 0; i < cloud_size; ++i) {
    if (i % mapper_params_->point_filter_num == 0) {
      current_feature_data_.cloud->push_back(_cloud_data.cloud->points[i]);
    }
  }
}

bool ClinsCore::UpdateKeyFrame() {
  // 缓存当前帧
  cache_feature_container_[current_feature_data_.scan_start_timestamp_ns] =
      current_feature_data_;

  // 更新关键帧
  for (auto iter = cache_feature_container_.begin();
       iter != cache_feature_container_.end();) {
    // 删除非关键帧
    if (!CheckKeyFrame(iter->first)) {
      if (iter->second.scan_end_timestamp_ns > trajectory_->GetActiveTime()) {
        break;
      } else {
        cache_feature_container_.erase(iter++);
        continue;
      }
    }
    local_feature_container_.push_back(iter->second);
    cache_feature_container_.erase(iter++);
    keyframe_update_flag_ = true;
  }

  return keyframe_update_flag_;
}

void ClinsCore::UpdateLocalFeatureMap() {}

bool ClinsCore::CheckKeyFrame(int64_t _timestamp_ns) {
  if (active_lidar_map_session_->GetMapUnitSize() == 0) {
    return true;
  }

  if (_timestamp_ns > trajectory_->GetActiveTime()) {
    return false;
  }

  if (_timestamp_ns - last_keyframe_timestamp_ns_ >
      keyframe_adding_time_ns_threshold_) {
    return true;
  }

  SE3d pose_cur = trajectory_->GetSensorPose(_timestamp_ns, FrameType::LIDAR);
  SE3d delta_pose = last_keyframe_pose_inv_ * pose_cur;
  Eigen::AngleAxisd v(delta_pose.so3().unit_quaternion());

  double dist_meter = delta_pose.translation().norm();
  double angle_degree = v.angle() * (180. / M_PI);

  if (angle_degree > mapper_params_->keyframe_adding_angle_threshold ||
      dist_meter > mapper_params_->keyframe_adding_distance_threshold) {
    return true;
  }

  return false;
}

int ClinsCore::FeatureAssociation(const KRTPointCloudPtr& _feature_in_global) {
  size_t feature_size = _feature_in_global->size();
  feature_corr_container_.reserve(feature_size);
  int cor_cnt = 0;
  for (size_t i = 0; i < feature_size; i++) {
    KRTPoint point_in_world = _feature_in_global->points[i];

    std::vector<float> point_search_sqr_dis(5);

    auto& points_near = nearest_points_[i];

    // 最近邻搜索
    ikd_tree_.Nearest_Search(point_in_world, 5, points_near,
                             point_search_sqr_dis);
    feature_selected_flag_container_[i] =
        points_near.size() < 5 ? false
                               : point_search_sqr_dis[4] > 5 ? false : true;
    if (!feature_selected_flag_container_[i]) continue;

    KRTPoint point_in_body = current_feature_data_.cloud->points[i];
    Eigen::Vector3f p_body(point_in_body.x, point_in_body.y, point_in_body.z);
    Eigen::Vector4f plane_coeff;
    feature_selected_flag_container_[i] = false;
    if (EstimatePlane(points_near, 0.1f, plane_coeff)) {
      float pd2 = plane_coeff(0) * point_in_world.x +
                  plane_coeff(1) * point_in_world.y +
                  plane_coeff(2) * point_in_world.z + plane_coeff(3);
      float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s > 0.9) {
        feature_selected_flag_container_[i] = true;
        PointCorrespondence pc;
        pc.point_timestamp_ns = current_feature_data_.scan_start_timestamp_ns +
                                point_in_body.timestamp_ns;
        pc.point =
            Eigen::Vector3d(point_in_body.x, point_in_body.y, point_in_body.z);
        pc.geo_plane = plane_coeff.template cast<double>();
        pc.distance_to_plane = pd2;
        feature_corr_container_.push_back(pc);
        cor_cnt++;
      }
    }
  }
  feature_corr_container_.resize(cor_cnt);

  return cor_cnt;
}

bool ClinsCore::EstimatePlane(const KPointVector& _points,
                              const float& _threshold,
                              Eigen::Vector4f& _plane_coeff) {
  Eigen::Matrix<float, 5, 3> A;
  Eigen::Matrix<float, 5, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < 5; j++) {
    A(j, 0) = _points[j].x;
    A(j, 1) = _points[j].y;
    A(j, 2) = _points[j].z;
  }

  Eigen::Vector3f norm_vec = A.colPivHouseholderQr().solve(b);

  float n = norm_vec.norm();
  _plane_coeff(0) = norm_vec(0) / n;
  _plane_coeff(1) = norm_vec(1) / n;
  _plane_coeff(2) = norm_vec(2) / n;
  _plane_coeff(3) = 1.0 / n;

  for (int j = 0; j < 5; j++) {
    if (fabs(_plane_coeff(0) * _points[j].x + _plane_coeff(1) * _points[j].y +
             _plane_coeff(2) * _points[j].z + _plane_coeff(3)) > _threshold) {
      return false;
    }
  }
  return true;
}

}  // namespace multi_sensor_mapping