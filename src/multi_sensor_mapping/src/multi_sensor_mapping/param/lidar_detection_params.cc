#include "multi_sensor_mapping/param/lidar_detection_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"
namespace multi_sensor_mapping {

LidarDetectionParams::LidarDetectionParams(std::string _name)
    : enable_detection(false),
      lidar_tag_num(1),
      num_points_for_plane(3),
      intensity_threshold(254.0),
      depth_threshold(0.4f),
      tag_size(0.6f),
      leaf_size(0.02f),
      layer_size(4) {
  name_ = _name;
}

void LidarDetectionParams::Load(std::string _path_of_yaml) {
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

  utils::YamlRead<bool>(config, "enable_detection", enable_detection, false);
  utils::YamlRead<int>(config, "lidar_tag_num", lidar_tag_num, 1);
  utils::YamlRead<int>(config, "num_points_for_plane", num_points_for_plane, 3);
  utils::YamlRead<float>(config, "intensity_threshold", intensity_threshold,
                         255.0);
  utils::YamlRead<float>(config, "depth_threshold", depth_threshold, 0.4);
  utils::YamlRead<float>(config, "tag_size", tag_size, 0.6);
  utils::YamlRead<float>(config, "leaf_size", leaf_size, 0.02);
  utils::YamlRead<int>(config, "layer_size", layer_size, 4);
}

void LidarDetectionParams::Print() {
  std::cout << "\033[1;32m----> Lidar Detection Params <----\033[0m"
            << std::endl;

  PrintLine("enable_detection", enable_detection);
  PrintLine("lidar_tag_num", lidar_tag_num);
  PrintLine("num_points_for_plane", num_points_for_plane);
  PrintLine("intensity_threshold", intensity_threshold);
  PrintLine("depth_threshold", depth_threshold);
  PrintLine("tag_size", tag_size);
  PrintLine("leaf_size", leaf_size);
  PrintLine("layer_size", layer_size);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping