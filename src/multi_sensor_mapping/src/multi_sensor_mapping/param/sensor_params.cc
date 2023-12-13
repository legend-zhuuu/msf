#include "multi_sensor_mapping/param/sensor_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

void SensorParams::Load(std::string _path_of_yaml) {
  YAML::Node config;
  try {
    config = YAML::LoadFile(_path_of_yaml);
  } catch (...) {
    AERROR_F(
        "The format of config file [ %s ] is wrong, Please check (e.g. "
        "indentation).",
        _path_of_yaml.c_str());
    return;
  }

  // Read Lidar Params
  bool set_marjor_lidar_flag = false;
  YAML::Node lidar_config = utils::YamlSubNodeAbort(config, "lidar");
  for (size_t i = 0; i < lidar_config.size(); i++) {
    bool use_flag = lidar_config[i]["use"].as<bool>();
    if (use_flag) {
      LidarParam param;
      param.name = StrToLidarName(lidar_config[i]["name"].as<std::string>());
      param.topic = lidar_config[i]["topic"].as<std::string>();
      param.frame_id = lidar_config[i]["frame"].as<std::string>();
      param.format =
          StrToLidarDataFormat(lidar_config[i]["format"].as<std::string>());
      param.purpose =
          StrToSensorPurpose(lidar_config[i]["purpose"].as<std::string>());
      param.use_flag = true;
      param.major_flag = lidar_config[i]["major"].as<bool>();
      lidar_param.push_back(param);

      if (param.major_flag) {
        major_lidar_ = param.name;
        set_marjor_lidar_flag = true;
      }
    }
  }

  if (lidar_param.empty()) {
    AWARN_F("[SensorParams] Sensor parameter error! Lidar not used");
  } else {
    // 若用户没有设置则默认第一个激光雷达为主激光雷达
    if (!set_marjor_lidar_flag) {
      AINFO_F(
          "No Major Lidar is set. The first lidar is the major lidar by "
          "default");
      major_lidar_ = lidar_param.front().name;
    }
  }

  // Read IMU
  YAML::Node imu_config = utils::YamlSubNodeAbort(config, "imu");
  imu_param.topic = imu_config["topic"].as<std::string>();
  imu_param.frame_id = imu_config["frame"].as<std::string>();
  imu_param.use_flag = imu_config["use"].as<bool>();

  // Read wheel
  YAML::Node wheel_config = utils::YamlSubNodeAbort(config, "wheel");
  wheel_param.topic = wheel_config["topic"].as<std::string>();
  wheel_param.frame_id = wheel_config["frame"].as<std::string>();
  wheel_param.use_flag = wheel_config["use"].as<bool>();

  // Read wheel
  YAML::Node gnss_config = utils::YamlSubNodeAbort(config, "gnss");
  gnss_param.topic = gnss_config["topic"].as<std::string>();
  gnss_param.frame_id = gnss_config["frame"].as<std::string>();
  gnss_param.use_flag = gnss_config["use"].as<bool>();
}

void SensorParams::Print() {
  std::cout << "\033[1;32m----> Sensor Params <----\033[0m" << std::endl;

  for (size_t i = 0; i < lidar_param.size(); i++) {
    if (!lidar_param[i].use_flag) {
      continue;
    }
    PrintLine("lidar_name", LidarNameToStr(lidar_param[i].name));
    PrintLine("lidar_topic", lidar_param[i].topic);
    PrintLine("frame_id", lidar_param[i].frame_id);
    PrintLine("lidar_format", LidarDataFormatToStr(lidar_param[i].format));
    PrintLine("lidar_purpose", SensorPurposeToStr(lidar_param[i].purpose));

    std::cout << BLUE << "--------------------------" << RESET << std::endl;
  }

  if (imu_param.use_flag) {
    PrintLine("imu_topic", imu_param.topic);
    PrintLine("frame_id", imu_param.frame_id);
  }

  std::cout << BLUE << "--------------------------" << RESET << std::endl;
  if (wheel_param.use_flag) {
    PrintLine("wheel_topic", wheel_param.topic);
    PrintLine("frame_id", wheel_param.frame_id);
  }
  std::cout << BLUE << "--------------------------" << RESET << std::endl;

  if (gnss_param.use_flag) {
    PrintLine("gnss_topic", gnss_param.topic);
    PrintLine("frame_id", gnss_param.frame_id);
  }

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

std::string SensorParams::MajorLidarTopic() {
  std::string lidar_topic = "";
  for (size_t i = 0; i < lidar_param.size(); i++) {
    if (lidar_param[i].major_flag) {
      lidar_topic = lidar_param[i].topic;
      break;
    }
  }

  return lidar_topic;
}

LidarDataFormat SensorParams::MajorLidarFormat() {
  LidarDataFormat lidar_format = LidarDataFormat::VAGUE;
  for (size_t i = 0; i < lidar_param.size(); i++) {
    if (lidar_param[i].major_flag) {
      lidar_format = lidar_param[i].format;
      break;
    }
  }

  return lidar_format;
}

std::string SensorParams::LIOLidarTopic() {
  std::string lidar_topic = "";
  for (size_t i = 0; i < lidar_param.size(); i++) {
    if (lidar_param[i].purpose == SensorPurpose::SP_LIO) {
      lidar_topic = lidar_param[i].topic;
      break;
    }
  }

  return lidar_topic;
}

LidarDataFormat SensorParams::LIOLidarFormat() {
  LidarDataFormat lidar_format = LidarDataFormat::VAGUE;
  for (size_t i = 0; i < lidar_param.size(); i++) {
    if (lidar_param[i].purpose == SensorPurpose::SP_LIO) {
      lidar_format = lidar_param[i].format;
      break;
    }
  }

  return lidar_format;
}

}  // namespace multi_sensor_mapping
