#include "multi_sensor_mapping/param/clins_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

ClinsParams::ClinsParams(std::string _name) { name_ = _name; }

void ClinsParams::Load(std::string _path_of_yaml) {
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

  utils::YamlRead<double>(config, "knot_distance", knot_distance, 0);
  utils::YamlRead<int>(config, "update_every_k_knot", update_every_k_knot, 4);
  utils::YamlRead<int>(config, "point_filter_num", point_filter_num, 3);

  utils::YamlRead<double>(config, "lidar_min_distance", lidar_min_distance, 1);
  utils::YamlRead<double>(config, "lidar_max_distance", lidar_max_distance,
                          100);

  utils::YamlRead<double>(config, "feature_leaf_size", feature_leaf_size, 0);

  utils::YamlRead<double>(config, "keyframe_adding_time_threshold",
                          keyframe_adding_time_threshold, 30);
  utils::YamlRead<double>(config, "keyframe_adding_distance_threshold",
                          keyframe_adding_distance_threshold, 1.0);
  utils::YamlRead<double>(config, "keyframe_adding_angle_threshold",
                          keyframe_adding_angle_threshold, 1.0);
  utils::YamlRead<double>(config, "keyframe_mapping_downsample_size",
                          keyframe_mapping_downsample_size, 1.0);

  utils::YamlRead<double>(config, "lidar_weight", lidar_weight, 1.0);
  utils::YamlRead<double>(config, "global_velocity_weight",
                          global_velocity_weight, 1.0);

  utils::YamlRead<double>(config, "initial_window_length",
                          initial_window_length, 0.75);
  utils::YamlRead<double>(config, "imu_excite_threshold", imu_excite_threshold,
                          0.12);
  utils::YamlRead<double>(config, "gravity_norm", gravity_norm, 9.8);
  utils::YamlRead<double>(config, "imu_frequency", imu_frequency, 200);
  utils::YamlRead<double>(config, "imu_accel_noise", imu_accel_noise, 0.0096);
  utils::YamlRead<double>(config, "imu_gyro_noise", imu_gyro_noise, 0.0034);
  utils::YamlRead<double>(config, "imu_accel_bias_noise", imu_accel_bias_noise,
                          0.00064);
  utils::YamlRead<double>(config, "imu_gyro_bias_noise", imu_gyro_bias_noise,
                          0.00035);
}

void ClinsParams::Print() {
  std::cout << "\033[1;32m----> CLINS Params <----\033[0m" << std::endl;
  PrintLine("knot_distance", knot_distance);
  PrintLine("update_every_k_knot", update_every_k_knot);
  PrintLine("point_filter_num", point_filter_num);
  PrintLine("feature_leaf_size", feature_leaf_size);
  PrintLine("keyframe_adding_time_threshold", keyframe_adding_time_threshold);
  PrintLine("keyframe_adding_distance_threshold",
            keyframe_adding_distance_threshold);
  PrintLine("keyframe_adding_angle_threshold", keyframe_adding_angle_threshold);
  PrintLine("keyframe_mapping_downsample_size",
            keyframe_mapping_downsample_size);
  PrintLine("lidar_weight", lidar_weight);
  PrintLine("global_velocity_weight", global_velocity_weight);
  PrintLine("initial_window_length", initial_window_length);
  PrintLine("imu_excite_threshold", imu_excite_threshold);

  PrintLine("gravity_norm", gravity_norm);
  PrintLine("imu_frequency", imu_frequency);
  PrintLine("imu_accel_noise", imu_accel_noise);
  PrintLine("imu_gyro_noise", imu_gyro_noise);
  PrintLine("imu_accel_bias_noise", imu_accel_bias_noise);
  PrintLine("imu_gyro_bias_noise", imu_gyro_bias_noise);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping