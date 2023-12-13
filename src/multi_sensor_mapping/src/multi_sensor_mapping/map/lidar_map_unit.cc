#include "multi_sensor_mapping/map/lidar_map_unit.h"

namespace multi_sensor_mapping {

int LidarMapUnit::next_unit_id = 0;

LidarMapUnit::LidarMapUnit()
    : timestamp_(0),
      full_cloud_(new CloudType),
      corner_feature_cloud_ds_(new CloudType),
      surface_feature_cloud_ds_(new CloudType),
      position_(Eigen::Vector3d(0, 0, 0)),
      orientation_(Eigen::Quaterniond::Identity()),
      LENU_position_(Eigen::Vector3d(0, 0, 0)),
      LENU_orientation_(Eigen::Quaterniond::Identity()),
      bad_flag_(false),
      ground_coeff_(Eigen::Vector4d(0, 0, 0, 0)),
      slice_attribute_(Eigen::Vector3d(20, 0.3, 0.4)),
      anchor_point_id_(-1) {
  unit_id_ = next_unit_id++;
}

LidarMapUnit::LidarMapUnit(double _timestamp, CloudTypePtr _full_cloud,
                           CloudTypePtr _corner_cloud,
                           CloudTypePtr _surface_cloud, Eigen::Matrix4d &_pose)
    : timestamp_(_timestamp),
      corner_feature_cloud_ds_(_corner_cloud),
      surface_feature_cloud_ds_(_surface_cloud),
      LENU_position_(Eigen::Vector3d(0, 0, 0)),
      LENU_orientation_(Eigen::Quaterniond::Identity()),
      bad_flag_(false),
      ground_coeff_(Eigen::Vector4d(0, 0, 0, 0)),
      slice_attribute_(Eigen::Vector3d(20, 0.3, 0.4)),
      anchor_point_id_(-1) {
  unit_id_ = next_unit_id++;
  full_cloud_.reset(new CloudType());
  *full_cloud_ = *_full_cloud;
  position_ = _pose.block<3, 1>(0, 3);
  orientation_ = Eigen::Quaterniond(_pose.block<3, 3>(0, 0));
}

LidarMapUnit::LidarMapUnit(double _timestamp, CloudTypePtr _full_cloud,
                           Eigen::Matrix4d &_pose)
    : timestamp_(_timestamp),
      LENU_position_(Eigen::Vector3d(0, 0, 0)),
      LENU_orientation_(Eigen::Quaterniond::Identity()),
      bad_flag_(false),
      ground_coeff_(Eigen::Vector4d(0, 0, 0, 0)),
      slice_attribute_(Eigen::Vector3d(20, 0.3, 0.4)),
      anchor_point_id_(-1) {
  unit_id_ = next_unit_id++;
  full_cloud_.reset(new CloudType());
  *full_cloud_ = *_full_cloud;
  position_ = _pose.block<3, 1>(0, 3);
  orientation_ = Eigen::Quaterniond(_pose.block<3, 3>(0, 0));
}

LidarMapUnit::LidarMapUnit(double _timestamp, CloudTypePtr _full_cloud,
                           CloudTypePtr _corner_cloud,
                           CloudTypePtr _surface_cloud, Eigen::Matrix4d &_pose,
                           Eigen::Vector4d &_ground_coeff)
    : timestamp_(_timestamp),
      full_cloud_(_full_cloud),
      corner_feature_cloud_ds_(_corner_cloud),
      surface_feature_cloud_ds_(_surface_cloud),
      LENU_position_(Eigen::Vector3d(0, 0, 0)),
      LENU_orientation_(Eigen::Quaterniond::Identity()),
      bad_flag_(false),
      ground_coeff_(_ground_coeff),
      slice_attribute_(Eigen::Vector3d(20, 0.3, 0.4)),
      anchor_point_id_(-1) {
  unit_id_ = next_unit_id++;
  position_ = _pose.block<3, 1>(0, 3);
  orientation_ = Eigen::Quaterniond(_pose.block<3, 3>(0, 0));
}

Eigen::Matrix4d LidarMapUnit::GetPose() {
  Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
  t.block<3, 3>(0, 0) = orientation_.toRotationMatrix();
  t(0, 3) = position_[0];
  t(1, 3) = position_[1];
  t(2, 3) = position_[2];
  return t;
}

}  // namespace multi_sensor_mapping
