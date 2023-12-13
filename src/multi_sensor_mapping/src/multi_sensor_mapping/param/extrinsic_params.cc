#include "multi_sensor_mapping/param/extrinsic_params.h"

#include "multi_sensor_mapping/utils/utils_common.h"
#include "multi_sensor_mapping/utils/utils_geometry.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

Eigen::Matrix4d LoadMatrixFromYaml(const YAML::Node& _yaml) {
  double x, y, z, roll, pitch, yaw;
  utils::YamlRead<double>(_yaml, "x", x, 0);
  utils::YamlRead<double>(_yaml, "y", y, 0);
  utils::YamlRead<double>(_yaml, "z", z, 0);
  utils::YamlRead<double>(_yaml, "roll", roll, 0);
  utils::YamlRead<double>(_yaml, "pitch", pitch, 0);
  utils::YamlRead<double>(_yaml, "yaw", yaw, 0);

  return utils::XYZYPR2Transd(x, y, z, yaw, pitch, roll);
}

void ExtrinsicParams::Load(std::string _path_of_yaml) {
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

  // 读取Lidar 外参
  YAML::Node lidar_config = utils::YamlSubNodeAbort(config, "lidar");
  for (size_t i = 0; i < lidar_config.size(); i++) {
    LidarName lidar_name =
        StrToLidarName(lidar_config[i]["name"].as<std::string>());

    lidar_to_baselink[lidar_name] = LoadMatrixFromYaml(lidar_config[i]);
  }

  // 读取 IMU 外参
  YAML::Node imu_config = utils::YamlSubNodeAbort(config, "imu");
  imu_to_baselink = LoadMatrixFromYaml(imu_config);
  imu_to_baselink_translation_ = imu_to_baselink.block<3, 1>(0, 3);
  imu_to_baselink_rotation_ =
      Eigen::Quaterniond(imu_to_baselink.block<3, 3>(0, 0));

  // 读取 odom 外参
  if (utils::CheckYamlSubNode(config, "wheel")) {
    YAML::Node odom_config = utils::YamlSubNodeAbort(config, "wheel");
    wheel_to_baselink = LoadMatrixFromYaml(odom_config);
    wheel_to_baselink_translation_ = wheel_to_baselink.block<3, 1>(0, 3);
    wheel_to_baselink_rotation_ =
        Eigen::Quaterniond(wheel_to_baselink.block<3, 3>(0, 0));
  }

  // 读取 gnss 外参
  if (utils::CheckYamlSubNode(config, "gnss")) {
    YAML::Node gnss_config = utils::YamlSubNodeAbort(config, "gnss");
    gnss_to_baselink = LoadMatrixFromYaml(gnss_config);
    gnss_to_baselink_translation_ = gnss_to_baselink.block<3, 1>(0, 3);
    gnss_to_baselink_rotation_ =
        Eigen::Quaterniond(gnss_to_baselink.block<3, 3>(0, 0));
  }
}

void ExtrinsicParams::PrintSensorExtParam(std::string _sensor_name,
                                          const Eigen::Matrix4d& _matrix) {
  double x, y, z, roll, pitch, yaw;
  utils::Transd2XYZYPR(_matrix, x, y, z, yaw, pitch, roll);
  PrintLine("Sensor", _sensor_name);
  PrintLine("x", x);
  PrintLine("y", y);
  PrintLine("z", z);
  PrintLine("roll", roll);
  PrintLine("pitch", pitch);
  PrintLine("yaw", yaw);
}

void ExtrinsicParams::Print() {
  std::cout << "\033[1;32m----> Extrinsic Params <----\033[0m" << std::endl;

  for (auto iter = lidar_to_baselink.begin(); iter != lidar_to_baselink.end();
       iter++) {
    PrintSensorExtParam(LidarNameToStr(iter->first), iter->second);

    std::cout << BLUE << "--------------------------" << RESET << std::endl;
  }

  PrintSensorExtParam("IMU", imu_to_baselink);
  std::cout << BLUE << "--------------------------" << RESET << std::endl;

  PrintSensorExtParam("Wheel", wheel_to_baselink);
  std::cout << BLUE << "--------------------------" << RESET << std::endl;

  PrintSensorExtParam("Gnss", gnss_to_baselink);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

Eigen::Quaterniond ExtrinsicParams::GetLidarToBaseRotation(
    const LidarName& _lidar_name) {
  Eigen::Matrix4d mat = lidar_to_baselink[_lidar_name];
  return Eigen::Quaterniond(mat.block<3, 3>(0, 0));
}

}  // namespace multi_sensor_mapping
