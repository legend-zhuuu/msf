#include "multi_sensor_mapping/utils/data_manager.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/rslidar_msg/rslidar_packet.hpp"
#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

DataManager::DataManager(const std::shared_ptr<DataConverter>& _data_converter)
    : data_converter_(_data_converter),
      get_imu_pointer_(0),
      get_wheel_pointer_(0),
      get_gnss_data_pointer_(0),
      lidar_data_count_(0),
      lidar_start_time_(0) {}

bool DataManager::ReadDataFromRosbag(
    const std::string _bag_path,
    const std::shared_ptr<SensorParams>& _sensor_params,
    const double _start_time, const double _bag_durr) {
  rosbag::Bag bag;
  try {
    bag.open(_bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& error) {
    AWARN_F("[DataManager] Fail to open bag [ %s ]", error.what());
    return false;
  }

  rosbag::View view;
  std::vector<std::string> topics;

  topics.push_back(_sensor_params->MajorLidarTopic());
  if (_sensor_params->imu_param.use_flag) {
    topics.push_back(_sensor_params->imu_param.topic);
  }
  if (_sensor_params->wheel_param.use_flag) {
    topics.push_back(_sensor_params->wheel_param.topic);
  }
  if (_sensor_params->gnss_param.use_flag) {
    topics.push_back(_sensor_params->gnss_param.topic);
  }

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(_start_time);
  ros::Time time_finish = (_bag_durr < 0)
                              ? view_full.getEndTime()
                              : time_init + ros::Duration(_bag_durr);

  ros::Duration delta_durr = ros::Duration(0.1);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init - delta_durr,
                time_finish + delta_durr);

  for (rosbag::MessageInstance const m : view) {
    const std::string& type = m.getDataType();
    const std::string& topic = m.getTopic();

    if (type == "sensor_msgs/Imu" && topic == _sensor_params->imu_param.topic) {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      imu_data_sequence_.push_back(data_converter_->ConvertImuData(imu_msg));
    } else if (type == "nav_msgs/Odometry" &&
               topic == _sensor_params->wheel_param.topic) {
      nav_msgs::Odometry::ConstPtr odom_msg =
          m.instantiate<nav_msgs::Odometry>();
      odom_data_sequence_.push_back(data_converter_->ConvertOdomData(odom_msg));
    } else if (type == "sensor_msgs/NavSatFix" &&
               topic == _sensor_params->gnss_param.topic) {
      sensor_msgs::NavSatFix::ConstPtr gnss_msg =
          m.instantiate<sensor_msgs::NavSatFix>();
      gnss_data_sequence_.push_back(data_converter_->ConvertGnssData(gnss_msg));
    } else if (type == "sensor_msgs/PointCloud2" &&
               topic == _sensor_params->MajorLidarTopic()) {
      if (lidar_data_count_ == 0) {
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        lidar_start_time_ = lidar_msg->header.stamp.toSec();
      }
      lidar_data_count_++;
    } else if (type == "rslidar_msg/RslidarPacket" &&
               topic == _sensor_params->MajorLidarTopic()) {
      if (lidar_data_count_ == 0) {
        rslidar_msg::RslidarPacket::ConstPtr packet_msg =
            m.instantiate<rslidar_msg::RslidarPacket>();
        lidar_start_time_ = packet_msg->header.stamp.toSec();
      }
      lidar_data_count_++;
    }
  }

  AINFO_F("[DataManager] Loading rosbag completed");
  AINFO_F(
      "[DataManager] Lidar message : %zu , Imu message : %zu , Wheel message : "
      "%zu , GNSS message : "
      "%zu ",
      lidar_data_count_, imu_data_sequence_.size(), odom_data_sequence_.size(),
      gnss_data_sequence_.size());

  return true;
}

bool DataManager::ReadDataFromRosbag2(
    const std::string _bag_path,
    const std::shared_ptr<SensorParams>& _sensor_params,
    const double _start_time, const double _bag_durr) {
  rosbag::Bag bag;
  try {
    bag.open(_bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& error) {
    AWARN_F("[DataManager] Fail to open bag [ %s ]", error.what());
    return false;
  }

  rosbag::View view;
  std::vector<std::string> topics;

  topics.push_back(_sensor_params->MajorLidarTopic());
  if (_sensor_params->imu_param.use_flag) {
    topics.push_back(_sensor_params->imu_param.topic);
  }
  if (_sensor_params->wheel_param.use_flag) {
    topics.push_back(_sensor_params->wheel_param.topic);
  }
  if (_sensor_params->gnss_param.use_flag) {
    topics.push_back(_sensor_params->gnss_param.topic);
  }

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(_start_time);
  ros::Time time_finish = (_bag_durr < 0)
                              ? view_full.getEndTime()
                              : time_init + ros::Duration(_bag_durr);

  ros::Duration delta_durr = ros::Duration(0.1);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init - delta_durr,
                time_finish + delta_durr);

  for (rosbag::MessageInstance const m : view) {
    const std::string& type = m.getDataType();
    const std::string& topic = m.getTopic();

    if (type == "sensor_msgs/Imu" && topic == _sensor_params->imu_param.topic) {
      sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      imu_data2_sequence_.push_back(data_converter_->ConvertImuData2(imu_msg));
    } else if (type == "sensor_msgs/PointCloud2" &&
               topic == _sensor_params->MajorLidarTopic()) {
      if (lidar_data_count_ == 0) {
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        lidar_start_time_ = lidar_msg->header.stamp.toSec();
      }
      lidar_data_count_++;
    } else if (type == "rslidar_msg/RslidarPacket" &&
               topic == _sensor_params->MajorLidarTopic()) {
      if (lidar_data_count_ == 0) {
        rslidar_msg::RslidarPacket::ConstPtr packet_msg =
            m.instantiate<rslidar_msg::RslidarPacket>();
        lidar_start_time_ = packet_msg->header.stamp.toSec();
      }
      lidar_data_count_++;
    }
  }

  AINFO_F("[DataManager] Loading rosbag completed");
  AINFO_F(
      "[DataManager] Lidar message : %zu , Imu message : %zu , Wheel message : "
      "%zu , GNSS message : "
      "%zu ",
      lidar_data_count_, imu_data2_sequence_.size(), odom_data_sequence_.size(),
      gnss_data_sequence_.size());

  return true;
}

bool DataManager::GetImuData(IMUData& imu_data, int index) {
  if (index >= imu_data_sequence_.size()) {
    AWARN_F("[DataManager] GetImuData failed, the index is too large");
    return false;
  }

  imu_data = imu_data_sequence_[index];
  return true;
}

bool DataManager::GetImuData(IMUData2& imu_data, int index) {
  if (index >= imu_data2_sequence_.size()) {
    AWARN_F("[DataManager] GetImuData failed, the index is too large");
    return false;
  }

  imu_data = imu_data2_sequence_[index];
  return true;
}

bool DataManager::GetImuDataInterval(std::vector<IMUData>& imu_vec,
                                     double begin_time, double end_time) {
  for (; get_imu_pointer_ < imu_data_sequence_.size(); get_imu_pointer_++) {
    if ((imu_data_sequence_[get_imu_pointer_].timestamp >= begin_time)) {
      imu_vec.push_back(imu_data_sequence_[get_imu_pointer_]);
    }

    if (imu_data_sequence_[get_imu_pointer_].timestamp > end_time) {
      break;
    }
  }
  if (imu_vec.empty()) {
    return false;
  }
  return true;
}

bool DataManager::GetImuDataInterval(std::vector<IMUData2>& imu_vec,
                                     int64_t _begin_time_ns,
                                     int64_t _end_time_ns) {
  for (; get_imu_pointer_ < imu_data2_sequence_.size(); get_imu_pointer_++) {
    if ((imu_data2_sequence_[get_imu_pointer_].timestamp_ns >=
         _begin_time_ns)) {
      imu_vec.push_back(imu_data2_sequence_[get_imu_pointer_]);
    }

    if (imu_data2_sequence_[get_imu_pointer_].timestamp_ns > _end_time_ns) {
      break;
    }
  }
  if (imu_vec.empty()) {
    return false;
  }
  return true;
}

void DataManager::LidarImuDataAlignment(double& _start_time,
                                        Eigen::Quaterniond& _imu_orientation) {
  _start_time = lidar_start_time_;
  if (imu_data_sequence_.front().timestamp > lidar_start_time_) {
    _imu_orientation = imu_data_sequence_.front().orientation;
    return;
  }

  for (size_t j = 0; j < imu_data_sequence_.size(); j++) {
    if (imu_data_sequence_[j].timestamp > lidar_start_time_) {
      Eigen::Quaterniond q_back = imu_data_sequence_[j - 1].orientation;
      Eigen::Quaterniond q_front = imu_data_sequence_[j].orientation;
      double q_ratio =
          (lidar_start_time_ - imu_data_sequence_[j - 1].timestamp) /
          (imu_data_sequence_[j].timestamp -
           imu_data_sequence_[j - 1].timestamp);
      _imu_orientation = q_back.slerp(q_ratio, q_front);

      return;
    }
  }
}

void DataManager::LidarImuData2Alignment(IMUState& _imu_state,
                                         IMUData2& _imu_data) {
  int64_t lidar_start_time_ns = lidar_start_time_ * 1e9;
  if (imu_data2_sequence_.front().timestamp_ns > lidar_start_time_ns) {
    _imu_state.timestamp_ns = imu_data2_sequence_.front().timestamp_ns;
    _imu_state.orientation =
        imu_data2_sequence_.front().orientation.unit_quaternion();
    _imu_data = imu_data2_sequence_.front();
    return;
  }

  for (size_t j = 0; j < imu_data2_sequence_.size(); j++) {
    if (imu_data2_sequence_[j].timestamp_ns > lidar_start_time_ns) {
      _imu_state.timestamp_ns = imu_data2_sequence_[j].timestamp_ns;
      Eigen::Quaterniond q_back =
          imu_data2_sequence_[j - 1].orientation.unit_quaternion();
      Eigen::Quaterniond q_front =
          imu_data2_sequence_[j].orientation.unit_quaternion();
      double q_ratio =
          float(lidar_start_time_ - imu_data2_sequence_[j - 1].timestamp_ns) /
          float(imu_data2_sequence_[j].timestamp_ns -
                imu_data2_sequence_[j - 1].timestamp_ns);
      _imu_state.orientation = q_back.slerp(q_ratio, q_front);
      _imu_data = imu_data2_sequence_[j];
      return;
    }
  }
}

size_t DataManager::GetLidarDataCount() { return lidar_data_count_; }

double DataManager::GetLidarStartTime() { return lidar_start_time_; }

}  // namespace multi_sensor_mapping