#include "multi_sensor_mapping/param/fast_lio_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

FastLIOParams::FastLIOParams(std::string _name) { name_ = _name; }

void FastLIOParams::Load(std::string _path_of_yaml) {
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

  utils::YamlRead<int>(config, "point_filter_num", point_filter_num, 3);
  utils::YamlRead<double>(config, "feature_ds_leaf_size", feature_ds_leaf_size,
                          0.5);
  utils::YamlRead<double>(config, "map_ds_leaf_size", map_ds_leaf_size, 0.5);
  utils::YamlRead<int>(config, "max_iterations", max_iterations, 3);
  utils::YamlRead<double>(config, "cube_length", cube_length, 1000);
  utils::YamlRead<double>(config, "delete_range", delete_range, 100.);

  utils::YamlRead<double>(config, "lidar_min_distance", lidar_min_distance,
                          1.0);
  utils::YamlRead<double>(config, "lidar_max_distance", lidar_max_distance,
                          100.);

  utils::YamlRead<double>(config, "gyro_cov", gyro_cov, 0.1);
  utils::YamlRead<double>(config, "accel_cov", accel_cov, 0.1);
  utils::YamlRead<double>(config, "gyro_bias_cov", gyro_bias_cov, 0.001);
  utils::YamlRead<double>(config, "accel_bias_cov", accel_bias_cov, 0.001);
  utils::YamlRead<double>(config, "keyframe_adding_threshold",
                          keyframe_adding_threshold, 1.0);
}

void FastLIOParams::Print() {
  std::cout << "\033[1;32m----> Fast-lio Params <----\033[0m" << std::endl;

  PrintLine("point_filter_num", point_filter_num);
  PrintLine("feature_ds_leaf_size", feature_ds_leaf_size);
  PrintLine("map_ds_leaf_size", map_ds_leaf_size);
  PrintLine("max_iterations", max_iterations);
  PrintLine("cube_length", cube_length);
  PrintLine("delete_range", delete_range);

  PrintLine("lidar_min_distance", lidar_min_distance);
  PrintLine("lidar_max_distance", lidar_max_distance);

  PrintLine("gyro_cov", gyro_cov);
  PrintLine("accel_cov", accel_cov);
  PrintLine("gyro_bias_cov", gyro_bias_cov);
  PrintLine("accel_bias_cov", accel_bias_cov);

  PrintLine("keyframe_adding_threshold", keyframe_adding_threshold);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping