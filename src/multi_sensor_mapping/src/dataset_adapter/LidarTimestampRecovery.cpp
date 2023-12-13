#include <dataset_adapter/LidarTimestampRecovery.h>
#include <multi_sensor_localization/utils/UtilsGeometry.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

namespace dataset_adapter {

LidarTimestampRecovery::LidarTimestampRecovery() {}

void LidarTimestampRecovery::SetInputCloud(
    const VelRTPointCloudPtr& raw_cloud) {
  double start_angle = 0, travelled_angle = 0;
  GetAngleOfScan(raw_cloud, start_angle, travelled_angle);
  start_point_angle_vec_.push_back(start_angle);
  rotation_travelled_angle_vec_.push_back(travelled_angle);
}

void LidarTimestampRecovery::RecoverTimestamp(int index,
                                              VelRTPointCloudPtr& raw_cloud) {
  double start_angle = correct_start_angle_vec_[index];
  RecoverPointCloudTimestmap(raw_cloud, start_angle);
}

void LidarTimestampRecovery::Reset() {
  start_point_angle_vec_.clear();
  rotation_travelled_angle_vec_.clear();
  correct_start_angle_vec_.clear();
}

void LidarTimestampRecovery::CorrectStartAngleOfScan() {
  // 正常情况下一帧激光应该转的角度；小于该角度说明[一帧数据起始处]有部分激光数据缺失
  double desired_complete_angle = 360 - 10;
  {
    // 一帧激光转了几圈
    double circle_cnt = 0;
    for (const auto& angle : rotation_travelled_angle_vec_)
      circle_cnt += (angle / 360.);
    circle_cnt = circle_cnt / (double)rotation_travelled_angle_vec_.size();
    double circle_for_one_scan = round(circle_cnt);

    desired_complete_angle = circle_for_one_scan * 360 - 10;
  }

  int scan_size = start_point_angle_vec_.size();

  double selected_angle = 5;
  int scan_idx[2] = {0, scan_size - 1};
  while (fabs(start_point_angle_vec_[scan_idx[0]] - selected_angle) > 10.0)
    scan_idx[0]++;
  while (fabs(start_point_angle_vec_[scan_idx[1]] - selected_angle) > 10.0)
    scan_idx[1]--;
  double scan_angle[2] = {start_point_angle_vec_[scan_idx[0]], 0};

  double rotation_travelled = 0;
  {
    RotationTravelledClockwise(scan_angle[0], true);
    for (int i = scan_idx[0]; i <= scan_idx[1]; ++i) {
      rotation_travelled =
          RotationTravelledClockwise(start_point_angle_vec_[i]);
    }
  }
  scan_angle[1] = -rotation_travelled;

  double line_k =
      (scan_angle[1] - scan_angle[0]) / (double)(scan_idx[1] - scan_idx[0]);
  double line_b = 0.5 * (scan_angle[0] - line_k * scan_idx[0]) +
                  0.5 * (scan_angle[1] - line_k * scan_idx[1]);

  // 估计一帧激光平均旋转角度
  int good_scan_cnt = 0;
  double good_scan_travelled = 0;

  correct_start_angle_vec_.reserve(start_point_angle_vec_.size());
  for (int k = 0; k < scan_size; ++k) {
    double correct_angle = -1000;
    if (rotation_travelled_angle_vec_[k] > desired_complete_angle) {
      good_scan_travelled += rotation_travelled_angle_vec_[k];
      good_scan_cnt++;
    } else {
      correct_angle =
          multi_sensor_localization::utils::NormalizeDeg(line_k * k + line_b);
      // 矫正结果与初始值相距太远的,放弃矫正
      if (ClockwiseAngle(correct_angle, start_point_angle_vec_[k]) > 120)
        correct_angle = -1000;
    }
    correct_start_angle_vec_.push_back(correct_angle);
  }

  one_scan_angle_ = good_scan_travelled / (double)good_scan_cnt;
  // vlp-16 固有参数
  static double dT = 55.296 * 24 * 76 * 1e-6;
  angle2time_ = dT / one_scan_angle_;

  std::cout << "one_scan_angle : " << one_scan_angle_ << std::endl;
}

void LidarTimestampRecovery::RecoverPointCloudTimestmap(
    VelRTPointCloudPtr raw_cloud, double start_angle) {
  bool is_first_horizon_angle = true;
  for (size_t i = 0; i < raw_cloud->size(); i++) {
    auto& raw_point = raw_cloud->points[i];
    if (!pcl_isfinite(raw_point.x)) continue;

    /// 计算横向angle
    double horizon_angle = atan2(raw_point.y, raw_point.x) * 180.0 / M_PI;
    if (is_first_horizon_angle) {
      is_first_horizon_angle = false;
      if (start_angle > -360) {
        RotationTravelledClockwise(start_angle, true);
        // std::cout << "start_angle/horizon_angle: " << start_angle << "/"
        //           << horizon_angle << std::endl;
      } else {
        RotationTravelledClockwise(horizon_angle, true);
      }
    }
    double rotation_travelled = RotationTravelledClockwise(horizon_angle);

    raw_point.time = rotation_travelled * angle2time_;
  }
}

void LidarTimestampRecovery::GetAngleOfScan(const VelRTPointCloudPtr& raw_cloud,
                                            double& start_angle,
                                            double& rotation_travelled) {
  // debug
  double max_angle = 0;
  // 起始位置标记
  bool is_first_horizon_angle = true;
  rotation_travelled = 0;
  for (size_t i = 0; i < raw_cloud->size(); i++) {
    auto& raw_point = raw_cloud->points[i];
    if (!pcl_isfinite(raw_point.x)) continue;

    double depth = sqrt(raw_point.x * raw_point.x + raw_point.y * raw_point.y +
                        raw_point.z * raw_point.z);
    if (depth < 0.05) continue;

    /// 计算横向angle
    double horizon_angle = atan2(raw_point.y, raw_point.x) * 180.0 / M_PI;
    if (is_first_horizon_angle) {
      is_first_horizon_angle = false;
      RotationTravelledClockwise(horizon_angle, true);
      start_angle = horizon_angle;
    }
    rotation_travelled = RotationTravelledClockwise(horizon_angle);

    max_angle = max_angle > rotation_travelled ? max_angle : rotation_travelled;
  }
}

double LidarTimestampRecovery::ClockwiseAngle(double bef_angle,
                                              double after_angle) {
  // atan2(py, px)
  // 结果为正表示从 X 轴逆时针旋转的角度 0 -> 180
  // 结果为负表示从 X 轴顺时针旋转的角度 0 -> -180

  // 1象限-1象限 = 45 - 30
  // 1象限-4象限 = 45 - (-30) = 75
  // 1象限-3象限 = 45 - (-120) = 165
  // 1象限-2象限 = 45 - 120 = -75 + 360 = 285

  // 4象限-4象限 = -45 - (-30) = -15 + 360
  // 4象限-3象限 = -45 - (-120) = 75
  // 4象限-2象限 = -45 - 120 = -165 + 360
  // 4象限-1象限 = -45 - 30 = -75 + 360
  double d_angle = bef_angle - after_angle;
  if (d_angle < 0) d_angle += 360;

  return d_angle;
}

double LidarTimestampRecovery::RotationTravelledClockwise(double now_angle,
                                                          bool reset_cnt) {
  static double start_angle_degree = 0;
  static bool half_rot_pointer = false;
  static double rot_circle_cnt = 0;

  if (reset_cnt) {
    start_angle_degree = now_angle;
    half_rot_pointer = false;
    rot_circle_cnt = 0;

    return 0;
  } else {
    double d_angle = ClockwiseAngle(start_angle_degree, now_angle);
    // 走过一半
    if (d_angle > 100 && d_angle < 270) {
      half_rot_pointer = true;
    }
    // 走过一圈
    if (half_rot_pointer && d_angle < 80) {
      half_rot_pointer = false;
      rot_circle_cnt += 360;
    }

    return rot_circle_cnt + d_angle;  // rot_travelled
  }
}

void LidarTimestampRecovery::LineFitting(const std::vector<double>& point,
                                         Eigen::Vector2d& line_k_b) {
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = point.size();
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(point.size());
  for (int i = 0; i < (int)point.size(); ++i) {
    (*cloud)[i].x = i;
    (*cloud)[i].y = point[i];
    (*cloud)[i].z = 0;
  }

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(
      new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();

  // std::vector<int> inliers;
  // ransac.getInliers(inliers);

  // point_on_line; line_direction
  Eigen::VectorXf coef = Eigen::VectorXf::Zero(6, 1);
  ransac.getModelCoefficients(coef);

  line_k_b[0] = coef[4] / coef[3];
  line_k_b[1] = coef[1] - coef[4] / coef[3] * coef[0];
}

}  // namespace dataset_adapter
