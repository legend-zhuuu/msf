#include "multi_sensor_mapping/frontend/lidar/scan_undistortion.h"

#include "multi_sensor_mapping/utils/utils_geometry.h"

namespace multi_sensor_mapping {

ScanUndistortion::ScanUndistortion(bool _correct_translation_distortion,
                                   bool _correct_rotation_distortion,
                                   bool _use_imu_orientation)
    : correct_translation_distortion_(_correct_translation_distortion),
      correct_rotation_distortion_(_correct_rotation_distortion),
      use_imu_data_(false),
      use_odom_data_(false),
      use_imu_orientation_(_use_imu_orientation),
      scan_start_time_(-1),
      scan_end_time_(-1),
      imu_avaliable_flag_(false),
      odom_avaliable_flag_(false) {
  CheckParameters();

  for (int i = 0; i < queue_length_; i++) {
    imu_deskew_rot_x_[i] = 0;
    imu_deskew_rot_y_[i] = 0;
    imu_deskew_rot_z_[i] = 0;
    imu_deskew_time_[i] = 0;

    odom_deskew_pos_x_[i] = 0;
    odom_deskew_pos_y_[i] = 0;
    odom_deskew_pos_z_[i] = 0;
    odom_deskew_time_[i] = 0;
  }
}

void ScanUndistortion::InputImuData(const IMUData &_imu_data) {
  if (!use_imu_data_) return;
  imu_data_queue_.push_back(_imu_data);
}

void ScanUndistortion::InputWheelData(const OdometryData &_odom_data) {
  if (!use_odom_data_) return;
  odom_data_queue_.push_back(_odom_data);
}

bool ScanUndistortion::UndistortScan(const double _scan_timestamp,
                                     VelRTPointCloudPtr _input_cloud,
                                     VelRTPointCloudPtr _output_cloud) {
  ResetParameters();
  _output_cloud->clear();
  scan_start_time_ = _scan_timestamp;
  scan_end_time_ = _scan_timestamp + _input_cloud->points.back().time;

  DeskewInfo();
  // trans_start_inverse_ = FindPose(_scan_timestamp).inverse();
  scan_start_pose_inv_ = FindRigid3(_scan_timestamp).inverse();

  for (size_t i = 0; i < _input_cloud->points.size(); i++) {
    VelRTPoint thisPoint = _input_cloud->points[i];

    double point_timestamp = scan_start_time_ + _input_cloud->points[i].time;
    thisPoint = DeskewPoint(thisPoint, point_timestamp);
    _output_cloud->push_back(thisPoint);
  }
  return true;
}

bool ScanUndistortion::UndistortScan(const double _scan_timestamp,
                                     RsRTPointCloudPtr _input_cloud,
                                     VelRTPointCloudPtr _output_cloud) {
  ResetParameters();
  _output_cloud->clear();
  scan_start_time_ = _scan_timestamp;
  scan_end_time_ = _input_cloud->points.back().timestamp;

  DeskewInfo();
  // trans_start_inverse_ = FindPose(_scan_timestamp).inverse();
  scan_start_pose_inv_ = FindRigid3(_scan_timestamp).inverse();

  for (size_t i = 0; i < _input_cloud->points.size(); i++) {
    VelRTPoint thisPoint;
    thisPoint.x = _input_cloud->points[i].x;
    thisPoint.y = _input_cloud->points[i].y;
    thisPoint.z = _input_cloud->points[i].z;
    thisPoint.intensity = _input_cloud->points[i].intensity;
    thisPoint.ring = _input_cloud->points[i].ring;
    thisPoint.time = float(_input_cloud->points[i].timestamp - _scan_timestamp);

    double point_timestamp = _input_cloud->points[i].timestamp;
    thisPoint = DeskewPoint(thisPoint, point_timestamp);
    _output_cloud->push_back(thisPoint);
  }

  return true;
}

bool ScanUndistortion::UndoScan(const double _scan_timestamp,
                                RsRTPointCloudPtr _input_cloud,
                                VelRTPointCloudPtr _output_cloud) {
  _output_cloud->clear();
  for (size_t i = 0; i < _input_cloud->points.size(); i++) {
    VelRTPoint thisPoint;
    thisPoint.x = _input_cloud->points[i].x;
    thisPoint.y = _input_cloud->points[i].y;
    thisPoint.z = _input_cloud->points[i].z;
    thisPoint.intensity = _input_cloud->points[i].intensity;
    thisPoint.ring = _input_cloud->points[i].ring;
    thisPoint.time = float(_input_cloud->points[i].timestamp - _scan_timestamp);
    _output_cloud->push_back(thisPoint);
  }

  return true;
}

void ScanUndistortion::CheckParameters() {
  if (correct_rotation_distortion_) {
    use_imu_data_ = true;
  } else {
    use_imu_data_ = false;
    use_odom_data_ = false;
    return;
  }
  if (correct_translation_distortion_) {
    use_odom_data_ = true;
  }
}

void ScanUndistortion::ResetParameters() {
  for (int i = 0; i < queue_length_; i++) {
    if (imu_deskew_time_[i] == 0) break;
    imu_deskew_rot_x_[i] = 0;
    imu_deskew_rot_y_[i] = 0;
    imu_deskew_rot_z_[i] = 0;
    imu_deskew_time_[i] = 0;
  }
  for (int i = 0; i < queue_length_; i++) {
    if (odom_deskew_time_[i] == 0) break;
    odom_deskew_pos_x_[i] = 0;
    odom_deskew_pos_y_[i] = 0;
    odom_deskew_pos_z_[i] = 0;
    odom_deskew_time_[i] = 0;
  }

  imu_pointer_cur_ = 0;
  odom_pointer_cur_ = 0;
  imu_avaliable_flag_ = false;
  odom_avaliable_flag_ = false;
}

VelRTPoint ScanUndistortion::DeskewPoint(VelRTPoint &_point, double _time) {
  if (!imu_avaliable_flag_) return _point;

  // Eigen::Affine3f point_pose = FindPose(_time);
  // Eigen::Affine3f d_trans = (trans_start_inverse_ * point_pose);
  Eigen::Matrix4f point_pose = FindRigid3(_time);
  Eigen::Matrix4f d_trans = scan_start_pose_inv_ * point_pose;
  VelRTPoint new_point;
  new_point.x = d_trans(0, 0) * _point.x + d_trans(0, 1) * _point.y +
                d_trans(0, 2) * _point.z + d_trans(0, 3);
  new_point.y = d_trans(1, 0) * _point.x + d_trans(1, 1) * _point.y +
                d_trans(1, 2) * _point.z + d_trans(1, 3);
  new_point.z = d_trans(2, 0) * _point.x + d_trans(2, 1) * _point.y +
                d_trans(2, 2) * _point.z + d_trans(2, 3);
  new_point.intensity = _point.intensity;
  new_point.ring = _point.ring;
  new_point.time = _point.time;

  return new_point;
}

void ScanUndistortion::TrimImuData() {
  if (scan_start_time_ < 0) return;
  while (!imu_data_queue_.empty()) {
    if (imu_data_queue_.front().timestamp < scan_start_time_ - 0.01)
      imu_data_queue_.pop_front();
    else {
      break;
    }
  }
}

void ScanUndistortion::TrimOdomData() {
  if (scan_start_time_ < 0) return;
  while (!odom_data_queue_.empty()) {
    if (odom_data_queue_.front().timestamp < scan_start_time_ - 0.01)
      odom_data_queue_.pop_front();
    else {
      break;
    }
  }
}

void ScanUndistortion::DeskewInfo() {
  ImuDeskewInfo();
  OdomDeskewInfo();
}

void ScanUndistortion::ImuDeskewInfo() {
  if (!correct_rotation_distortion_) return;
  TrimImuData();

  if (imu_data_queue_.empty()) return;
  if (imu_data_queue_.front().timestamp > scan_end_time_) return;

  imu_pointer_cur_ = 0;
  if (!use_imu_orientation_) {
    for (size_t i = 0; i < imu_data_queue_.size(); i++) {
      if (imu_data_queue_[i].timestamp > scan_end_time_ + 0.01) break;
      if (imu_pointer_cur_ == 0) {
        imu_deskew_rot_x_[0] = 0;
        imu_deskew_rot_y_[0] = 0;
        imu_deskew_rot_z_[0] = 0;
        imu_deskew_time_[0] = imu_data_queue_[i].timestamp;
        ++imu_pointer_cur_;
        continue;
      }

      double time_diff =
          imu_data_queue_[i].timestamp - imu_deskew_time_[imu_pointer_cur_ - 1];
      imu_deskew_rot_x_[imu_pointer_cur_] =
          imu_deskew_rot_x_[imu_pointer_cur_ - 1] +
          imu_data_queue_[i].gyro[0] * time_diff;
      imu_deskew_rot_y_[imu_pointer_cur_] =
          imu_deskew_rot_y_[imu_pointer_cur_ - 1] +
          imu_data_queue_[i].gyro[1] * time_diff;
      imu_deskew_rot_z_[imu_pointer_cur_] =
          imu_deskew_rot_z_[imu_pointer_cur_ - 1] +
          imu_data_queue_[i].gyro[2] * time_diff;
      imu_deskew_time_[imu_pointer_cur_] = imu_data_queue_[i].timestamp;
      ++imu_pointer_cur_;
    }
  } else {
    Eigen::Quaterniond start_q_inv;
    for (size_t i = 0; i < imu_data_queue_.size(); i++) {
      if (imu_data_queue_[i].timestamp > scan_end_time_ + 0.01) break;
      if (imu_pointer_cur_ == 0) {
        imu_deskew_rot_x_[0] = 0;
        imu_deskew_rot_y_[0] = 0;
        imu_deskew_rot_z_[0] = 0;
        imu_deskew_time_[0] = imu_data_queue_[i].timestamp;
        start_q_inv = imu_data_queue_[i].orientation.inverse();
        ++imu_pointer_cur_;
        continue;
      }

      // Eigen::Matrix3d delta_mat =
      //     (start_q_inv * imu_data_queue_[i].orientation).toRotationMatrix();

      // Eigen::Vector3d euler_rpy = utils::R2rpy(delta_mat);
      // imu_deskew_rot_x_[imu_pointer_cur_] = euler_rpy(0);
      // imu_deskew_rot_y_[imu_pointer_cur_] = euler_rpy(1);
      // imu_deskew_rot_z_[imu_pointer_cur_] = euler_rpy(2);

      Eigen::Quaterniond delta_q = start_q_inv * imu_data_queue_[i].orientation;
      tf::Quaternion orientation(delta_q.x(), delta_q.y(), delta_q.z(),
                                 delta_q.w());
      double roll, pitch, yaw;
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      imu_deskew_rot_x_[imu_pointer_cur_] = roll;
      imu_deskew_rot_y_[imu_pointer_cur_] = pitch;
      imu_deskew_rot_z_[imu_pointer_cur_] = yaw;

      imu_deskew_time_[imu_pointer_cur_] = imu_data_queue_[i].timestamp;
      ++imu_pointer_cur_;
    }
  }

  --imu_pointer_cur_;

  if (imu_pointer_cur_ <= 0) return;
  imu_avaliable_flag_ = true;
}

void ScanUndistortion::OdomDeskewInfo() {
  if (!correct_translation_distortion_) return;
  TrimOdomData();

  if (odom_data_queue_.empty()) return;
  if (odom_data_queue_.front().timestamp > scan_end_time_) return;

  odom_pointer_cur_ = 0;
  Eigen::Matrix4d start_pose;
  for (size_t i = 0; i < odom_data_queue_.size(); i++) {
    if (odom_data_queue_[i].timestamp > scan_end_time_ + 0.01) break;
    if (odom_pointer_cur_ == 0) {
      odom_deskew_pos_x_[0] = 0;
      odom_deskew_pos_y_[0] = 0;
      odom_deskew_pos_z_[0] = 0;
      odom_deskew_time_[0] = odom_data_queue_[i].timestamp;
      start_pose = odom_data_queue_[i].Matrix();
      ++odom_pointer_cur_;
      continue;
    }

    auto position_diff =
        (start_pose.inverse() * odom_data_queue_[i].Matrix()).block<3, 1>(0, 3);
    odom_deskew_pos_x_[odom_pointer_cur_] = position_diff(0);
    odom_deskew_pos_y_[odom_pointer_cur_] = position_diff(1);
    odom_deskew_pos_z_[odom_pointer_cur_] = position_diff(2);
    odom_deskew_time_[0] = odom_data_queue_[i].timestamp;
    ++odom_pointer_cur_;
  }

  --odom_pointer_cur_;
  if (odom_pointer_cur_ <= 0) return;
  odom_avaliable_flag_ = true;
}

void ScanUndistortion::FindRotation(const double point_time, float &rot_x,
                                    float &rot_y, float &rot_z) {
  rot_x = 0;
  rot_y = 0;
  rot_z = 0;

  if (!imu_avaliable_flag_) return;
  int deskew_pointer_front = 0;
  while (deskew_pointer_front < imu_pointer_cur_) {
    if (point_time < imu_deskew_time_[deskew_pointer_front]) break;
    deskew_pointer_front++;
  }

  if (point_time > imu_deskew_time_[deskew_pointer_front] ||
      deskew_pointer_front == 0) {
    rot_x = imu_deskew_rot_x_[deskew_pointer_front];
    rot_y = imu_deskew_rot_y_[deskew_pointer_front];
    rot_z = imu_deskew_rot_z_[deskew_pointer_front];
  } else {
    int deskew_pointer_back = deskew_pointer_front - 1;
    double ratio_front = (point_time - imu_deskew_time_[deskew_pointer_back]) /
                         (imu_deskew_time_[deskew_pointer_front] -
                          imu_deskew_time_[deskew_pointer_back]);
    double ratio_back = (imu_deskew_time_[deskew_pointer_front] - point_time) /
                        (imu_deskew_time_[deskew_pointer_front] -
                         imu_deskew_time_[deskew_pointer_back]);
    rot_x = imu_deskew_rot_x_[deskew_pointer_front] * ratio_front +
            imu_deskew_rot_x_[deskew_pointer_back] * ratio_back;
    rot_y = imu_deskew_rot_y_[deskew_pointer_front] * ratio_front +
            imu_deskew_rot_y_[deskew_pointer_back] * ratio_back;
    rot_z = imu_deskew_rot_z_[deskew_pointer_front] * ratio_front +
            imu_deskew_rot_z_[deskew_pointer_back] * ratio_back;
  }
}

void ScanUndistortion::FindPosition(const double point_time, float &pos_x,
                                    float &pos_y, float &pos_z) {
  pos_x = 0;
  pos_y = 0;
  pos_z = 0;

  if (!odom_avaliable_flag_) return;

  int deskew_pointer_front = 0;
  while (deskew_pointer_front < odom_pointer_cur_) {
    if (point_time < odom_deskew_time_[deskew_pointer_front]) break;
    deskew_pointer_front++;
  }

  if (point_time > odom_deskew_time_[deskew_pointer_front] ||
      deskew_pointer_front == 0) {
    pos_x = odom_deskew_pos_x_[deskew_pointer_front];
    pos_y = odom_deskew_pos_y_[deskew_pointer_front];
    pos_z = odom_deskew_pos_z_[deskew_pointer_front];
  } else {
    int deskew_pointer_back = deskew_pointer_front - 1;
    double ratio_front = (point_time - odom_deskew_time_[deskew_pointer_back]) /
                         (odom_deskew_time_[deskew_pointer_front] -
                          odom_deskew_time_[deskew_pointer_back]);
    double ratio_back = (odom_deskew_time_[deskew_pointer_front] - point_time) /
                        (odom_deskew_time_[deskew_pointer_front] -
                         odom_deskew_time_[deskew_pointer_back]);
    pos_x = odom_deskew_pos_x_[deskew_pointer_front] * ratio_front +
            odom_deskew_pos_x_[deskew_pointer_back] * ratio_back;
    pos_y = odom_deskew_pos_y_[deskew_pointer_front] * ratio_front +
            odom_deskew_pos_y_[deskew_pointer_back] * ratio_back;
    pos_z = odom_deskew_pos_z_[deskew_pointer_front] * ratio_front +
            odom_deskew_pos_z_[deskew_pointer_back] * ratio_back;
  }
}

Eigen::Affine3f ScanUndistortion::FindPose(const double _point_time) {
  float rot_x_cur, rot_y_cur, rot_z_cur;
  float pos_x_cur, pos_y_cur, pos_z_cur;

  FindRotation(_point_time, rot_x_cur, rot_y_cur, rot_z_cur);
  FindPosition(_point_time, pos_x_cur, pos_y_cur, pos_z_cur);

  return pcl::getTransformation(pos_x_cur, pos_y_cur, pos_z_cur, rot_x_cur,
                                rot_y_cur, rot_z_cur);
}

Eigen::Matrix4f toMatrix4f(float x, float y, float z, float roll, float pitch,
                           float yaw) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  float A = std::cos(yaw);
  float B = sin(yaw);
  float C = std::cos(pitch);
  float D = sin(pitch);
  float E = std::cos(roll);
  float F = sin(roll);
  float DE = D * E;
  float DF = D * F;

  matrix(0, 0) = A * C;
  matrix(0, 1) = A * DF - B * E;
  matrix(0, 2) = B * F + A * DE;
  matrix(1, 0) = B * C;
  matrix(1, 1) = A * E + B * DF;
  matrix(1, 2) = B * DE - A * F;
  matrix(2, 0) = -D;
  matrix(2, 1) = C * F;
  matrix(2, 2) = C * E;
  matrix(0, 3) = x;
  matrix(1, 3) = y;
  matrix(2, 3) = z;
  return matrix;
}

Eigen::Matrix4f ScanUndistortion::FindRigid3(const double _time) {
  float rot_x_cur, rot_y_cur, rot_z_cur;
  float pos_x_cur, pos_y_cur, pos_z_cur;

  FindRotation(_time, rot_x_cur, rot_y_cur, rot_z_cur);
  FindPosition(_time, pos_x_cur, pos_y_cur, pos_z_cur);

  return toMatrix4f(pos_x_cur, pos_y_cur, pos_z_cur, rot_x_cur, rot_y_cur,
                    rot_z_cur);
}

}  // namespace multi_sensor_mapping
