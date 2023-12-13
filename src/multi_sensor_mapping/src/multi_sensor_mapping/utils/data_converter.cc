#include "multi_sensor_mapping/utils/data_converter.h"

#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

DataConverter::DataConverter()
    : lidar_min_range_square_(1.0 * 1.0),
      lidar_max_range_square_(80.0 * 80.0) {}

DataConverter::DataConverter(double _min_range, double _max_range)
    : lidar_min_range_square_(_min_range * _min_range),
      lidar_max_range_square_(_max_range * _max_range) {}

IMUData DataConverter::ConvertImuData(
    const sensor_msgs::Imu::ConstPtr& _imu_msg) {
  IMUData data;
  data.timestamp = _imu_msg->header.stamp.toSec();
  data.gyro = Eigen::Vector3d(_imu_msg->angular_velocity.x,
                              _imu_msg->angular_velocity.y,
                              _imu_msg->angular_velocity.z);
  data.accel = Eigen::Vector3d(_imu_msg->linear_acceleration.x,
                               _imu_msg->linear_acceleration.y,
                               _imu_msg->linear_acceleration.z);
  data.orientation =
      Eigen::Quaterniond(_imu_msg->orientation.w, _imu_msg->orientation.x,
                         _imu_msg->orientation.y, _imu_msg->orientation.z);

  return data;
}

IMUData2 DataConverter::ConvertImuData2(
    const sensor_msgs::Imu::ConstPtr& _imu_msg) {
  IMUData2 data;
  data.timestamp_ns = _imu_msg->header.stamp.toNSec();
  data.gyro = Eigen::Vector3d(_imu_msg->angular_velocity.x,
                              _imu_msg->angular_velocity.y,
                              _imu_msg->angular_velocity.z);
  data.accel = Eigen::Vector3d(_imu_msg->linear_acceleration.x,
                               _imu_msg->linear_acceleration.y,
                               _imu_msg->linear_acceleration.z);
  data.orientation = SO3d(
      Eigen::Quaterniond(_imu_msg->orientation.w, _imu_msg->orientation.x,
                         _imu_msg->orientation.y, _imu_msg->orientation.z));

  return data;
}

OdometryData DataConverter::ConvertOdomData(
    const nav_msgs::Odometry::ConstPtr& _odom_msg) {
  OdometryData data;
  data.timestamp = _odom_msg->header.stamp.toSec();
  data.position = Eigen::Vector3d(_odom_msg->pose.pose.position.x,
                                  _odom_msg->pose.pose.position.y,
                                  _odom_msg->pose.pose.position.z);
  data.orientation = Eigen::Quaterniond(
      _odom_msg->pose.pose.orientation.w, _odom_msg->pose.pose.orientation.x,
      _odom_msg->pose.pose.orientation.y, _odom_msg->pose.pose.orientation.z);

  return data;
}

GNSSData DataConverter::ConvertGnssData(
    const sensor_msgs::NavSatFix::ConstPtr& _gnss_msg) {
  GNSSData gnss_data;

  gnss_data.timestamp = _gnss_msg->header.stamp.toSec();
  gnss_data.lla(0) = _gnss_msg->latitude;
  gnss_data.lla(1) = _gnss_msg->longitude;
  gnss_data.lla(2) = _gnss_msg->altitude;
  gnss_data.std(0) = sqrt(_gnss_msg->position_covariance[0]);
  gnss_data.std(1) = sqrt(_gnss_msg->position_covariance[4]);
  gnss_data.std(2) = sqrt(_gnss_msg->position_covariance[8]);

  gnss_data.status = (int)_gnss_msg->status.status;

  return gnss_data;
}

TimedCloudData DataConverter::ConvertRsCloudData(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData data;

  RsRTPointCloudPtr cloud(new RsRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);
  if (PreprocessCloud(cloud, data.cloud, data.timestamp)) {
    /// Ps: 激光断开重连后会出现时间戳不一致的情况
    if (fabs(data.timestamp - _cloud_msg->header.stamp.toSec()) < 0.3) {
      data.valid_flag = true;
    }
  }

  return data;
}

TimedCloudData2 DataConverter::ConvertRsCloudData2(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData2 data;

  RsRTPointCloudPtr cloud(new RsRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);

  if (PreprocessCloud(cloud, data.cloud, data.scan_start_timestamp_ns,
                      data.scan_end_timestamp_ns)) {
    /// TODO
  }

  return data;
}

TimedCloudData DataConverter::ConvertLegacyRsCloudData(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData data;

  LegacyRsRTPointCloudPtr cloud(new LegacyRsRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);
  if (PreprocessCloud(cloud, data.cloud, data.timestamp)) {
    /// Ps: 激光断开重连后会出现时间戳不一致的情况
    if (fabs(data.timestamp - _cloud_msg->header.stamp.toSec()) < 0.3) {
      data.valid_flag = true;
    }
  }

  return data;
}

TimedCloudData2 DataConverter::ConvertLegacyRsCloudData2(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData2 data;

  LegacyRsRTPointCloudPtr cloud(new LegacyRsRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);
  if (PreprocessCloud(cloud, data.cloud, data.scan_start_timestamp_ns,
                      data.scan_end_timestamp_ns)) {
    /// TODO
  }

  return data;
}

TimedCloudData DataConverter::ConvertVelCloudData(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData data;

  data.timestamp = _cloud_msg->header.stamp.toSec();

  VelRTPointCloudPtr cloud(new VelRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);
  VelRTPointCloudPtr cloud_in_track_frame(new VelRTPointCloud);
  if (PreprocessCloud(cloud, data.cloud, data.timestamp)) {
    data.valid_flag = true;
  }

  return data;
}

TimedCloudData DataConverter::ConvertHesaiCloudData(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData data;

  data.timestamp = _cloud_msg->header.stamp.toSec();
  HsRTPointCloudPtr cloud(new HsRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);
  if (PreprocessCloud(cloud, data.cloud, data.timestamp)) {
    /// Ps: 激光断开重连后会出现时间戳不一致的情况
    if (fabs(data.timestamp - _cloud_msg->header.stamp.toSec()) < 0.3) {
      data.valid_flag = true;
    }
  }

  return data;
}

TimedCloudData DataConverter::ConvertLivoxCloudData(
    const sensor_msgs::PointCloud2::ConstPtr _cloud_msg) {
  TimedCloudData data;

  data.timestamp = _cloud_msg->header.stamp.toSec();
  LvRTPointCloudPtr cloud(new LvRTPointCloud);
  pcl::fromROSMsg(*_cloud_msg, *cloud);
  if (PreprocessCloud(cloud, data.cloud, data.timestamp)) {
    /// Ps: 激光断开重连后会出现时间戳不一致的情况
    if (fabs(data.timestamp - _cloud_msg->header.stamp.toSec()) < 0.3) {
      data.valid_flag = true;
    }
  }

  return data;
}

bool DataConverter::PreprocessCloud(RsRTPointCloudPtr _cloud,
                                    VelRTPointCloudPtr& _output_cloud,
                                    double& _output_timestamp) {
  _output_cloud = VelRTPointCloudPtr(new VelRTPointCloud);
  _output_timestamp = _cloud->points.front().timestamp;

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  if (_cloud->is_dense == false) {
    for (size_t i = 0; i < cloud_size; i++) {
      // 需过滤NAN点
      RsRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.time = this_point.timestamp - _output_timestamp;

      /// 很老版本的速腾激光
      //      p.x *= 2;
      //      p.y *= 2;
      //      p.z *= 2;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  } else {
    for (size_t i = 0; i < cloud_size; i++) {
      RsRTPoint this_point = _cloud->points[i];

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.time = this_point.timestamp - _output_timestamp;

      /// 很老版本的速腾激光
      //      p.x *= 2;
      //      p.y *= 2;
      //      p.z *= 2;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

bool DataConverter::PreprocessCloud(RsRTPointCloudPtr _cloud,
                                    KRTPointCloudPtr& _output_cloud,
                                    int64_t& _scan_start_time,
                                    int64_t& _scan_end_time) {
  _output_cloud = KRTPointCloudPtr(new KRTPointCloud);
  _scan_start_time = int64_t(_cloud->points.front().timestamp * 1e9);
  _scan_end_time = int64_t(_cloud->points.back().timestamp * 1e9);

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  if (_cloud->is_dense == false) {
    for (size_t i = 0; i < cloud_size; i++) {
      // 需过滤NAN点
      RsRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      KRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.timestamp_ns = int64_t(this_point.timestamp * 1e9) - _scan_start_time;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  } else {
    for (size_t i = 0; i < cloud_size; i++) {
      RsRTPoint this_point = _cloud->points[i];

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      KRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.timestamp_ns = int64_t(this_point.timestamp * 1e9) - _scan_start_time;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

bool DataConverter::PreprocessCloud(LegacyRsRTPointCloudPtr _cloud,
                                    VelRTPointCloudPtr& _output_cloud,
                                    double& _output_timestamp) {
  _output_cloud = VelRTPointCloudPtr(new VelRTPointCloud);
  _output_timestamp = _cloud->points.front().timestamp;

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  if (_cloud->is_dense == false) {
    for (size_t i = 0; i < cloud_size; i++) {
      // 需过滤NAN点
      LegacyRsRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.time = this_point.timestamp - _output_timestamp;

      /// 很老版本的速腾激光
      //      p.x *= 2;
      //      p.y *= 2;
      //      p.z *= 2;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  } else {
    for (size_t i = 0; i < cloud_size; i++) {
      LegacyRsRTPoint this_point = _cloud->points[i];

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.time = this_point.timestamp - _output_timestamp;

      /// 很老版本的速腾激光
      //      p.x *= 2;
      //      p.y *= 2;
      //      p.z *= 2;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

bool DataConverter::PreprocessCloud(LegacyRsRTPointCloudPtr _cloud,
                                    KRTPointCloudPtr& _output_cloud,
                                    int64_t& _scan_start_time,
                                    int64_t& _scan_end_time) {
  _output_cloud = KRTPointCloudPtr(new KRTPointCloud);
  _scan_start_time = int64_t(_cloud->points.front().timestamp * 1e9);
  _scan_end_time = int64_t(_cloud->points.back().timestamp * 1e9);

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  if (_cloud->is_dense == false) {
    for (size_t i = 0; i < cloud_size; i++) {
      // 需过滤NAN点
      LegacyRsRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      KRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.timestamp_ns = int64_t(this_point.timestamp * 1e9) - _scan_start_time;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  } else {
    for (size_t i = 0; i < cloud_size; i++) {
      LegacyRsRTPoint this_point = _cloud->points[i];

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      KRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.timestamp_ns = int64_t(this_point.timestamp * 1e9) - _scan_start_time;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

bool DataConverter::PreprocessCloud(VelRTPointCloudPtr& _cloud,
                                    VelRTPointCloudPtr& _output_cloud,
                                    double& _output_timestamp) {
  _output_cloud = VelRTPointCloudPtr(new VelRTPointCloud);

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  /// Rslidar 转 Vlp格式(洗地机 航宇改的驱动)
  if (_cloud->height > 1) {
    for (size_t i = 0; i < cloud_size; ++i) {
      VelRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      // 根据距离过滤
      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      _output_cloud->push_back(this_point);
      valid_cloud_cnt++;
    }
    _output_timestamp -= 0.1;
  } else {
    /// VLP 原生驱动

    for (size_t i = 0; i < cloud_size; ++i) {
      VelRTPoint this_point = _cloud->points[i];

      // 根据距离过滤
      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      _output_cloud->push_back(this_point);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

bool DataConverter::PreprocessCloud(HsRTPointCloudPtr _cloud,
                                    VelRTPointCloudPtr& _output_cloud,
                                    double& _output_timestamp) {
  _output_cloud = VelRTPointCloudPtr(new VelRTPointCloud);
  _output_timestamp = _cloud->points.front().timestamp;

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  if (_cloud->is_dense == false) {
    for (size_t i = 0; i < cloud_size; i++) {
      // 需过滤NAN点
      HsRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.time = this_point.timestamp - _output_timestamp;

      /// 很老版本的速腾激光
      //      p.x *= 2;
      //      p.y *= 2;
      //      p.z *= 2;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  } else {
    for (size_t i = 0; i < cloud_size; i++) {
      HsRTPoint this_point = _cloud->points[i];

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.ring;
      p.time = this_point.timestamp - _output_timestamp;

      /// 很老版本的速腾激光
      //      p.x *= 2;
      //      p.y *= 2;
      //      p.z *= 2;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

bool DataConverter::PreprocessCloud(LvRTPointCloudPtr _cloud,
                                    VelRTPointCloudPtr& _output_cloud,
                                    double& _output_timestamp) {
  _output_cloud = VelRTPointCloudPtr(new VelRTPointCloud);
  _output_timestamp = _cloud->points.front().timestamp * 1e-9;

  size_t cloud_size = _cloud->size();
  size_t valid_cloud_cnt = 0;

  _output_cloud->reserve(cloud_size);

  if (_cloud->is_dense == false) {
    for (size_t i = 0; i < cloud_size; i++) {
      // 需过滤NAN点
      LvRTPoint this_point = _cloud->points[i];
      if (!std::isfinite(this_point.x) || !std::isfinite(this_point.y) ||
          !std::isfinite(this_point.z)) {
        continue;
      }

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.line;
      p.time = this_point.timestamp * 1e-9 - _output_timestamp;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  } else {
    for (size_t i = 0; i < cloud_size; i++) {
      LvRTPoint this_point = _cloud->points[i];

      float distance_sqr = PointDistanceSquare(this_point);
      if (distance_sqr < lidar_min_range_square_ ||
          distance_sqr > lidar_max_range_square_) {
        continue;
      }

      VelRTPoint p;
      p.x = this_point.x;
      p.y = this_point.y;
      p.z = this_point.z;
      p.intensity = this_point.intensity;
      p.ring = this_point.line;
      p.time = this_point.timestamp * 1e-9 - _output_timestamp;

      _output_cloud->push_back(p);
      valid_cloud_cnt++;
    }
  }

  _output_cloud->resize(valid_cloud_cnt);

  if (_output_cloud->size() < 1000) {
    AWARN_F("Lidar is maliciously blocked ! cloud size : %lu ",
            _output_cloud->size());
    return false;
  }

  return true;
}

}  // namespace multi_sensor_mapping