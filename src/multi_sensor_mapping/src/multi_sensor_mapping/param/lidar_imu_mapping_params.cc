#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

LidarImuMappingParams::LidarImuMappingParams(std::string _name)
    : num_ring(32),
      num_horizon(1800),
      number_of_cores(8),
      edge_threshold(1.0),
      surf_threshold(0.1),
      surf_feature_leaf_size(0.2),
      lidar_min_distance(1.0),
      lidar_max_distance(80.0),
      match_method(MatchMethod::MATCH_METHOD_FAST_LOAM),
      corner_ds_leaf_size(0.1),
      surface_ds_leaf_size(1.0),
      corner_feature_min_valid_num(10),
      surface_feature_min_valid_num(100),
      keyframe_search_radius(30),
      keyframe_adding_distance_threshold(1.0),
      keyframe_adding_angle_threshold(1.0),
      keyframe_mapping_downsample_size(1.0),
      auto_loop_closure(false),
      loop_closure_searching_distance(5.0),
      loop_closure_distance_interval(15.0),
      loop_closure_score_threshold(0.5),
      mono_layer_motion(false),
      initial_window_length(0.75),
      imu_excite_threshold(0.12),
      map_resolution(0.1) {
  name_ = _name;
}

void LidarImuMappingParams::Load(std::string _path_of_yaml) {
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

  utils::YamlRead<int>(config, "num_ring", num_ring, 32);
  utils::YamlRead<int>(config, "num_horizon", num_horizon, 1800);
  utils::YamlRead<int>(config, "num_of_cores", number_of_cores, 6);

  utils::YamlRead<double>(config, "edge_threshold", edge_threshold, 1.0);
  utils::YamlRead<double>(config, "surf_threshold", surf_threshold, 0.1);
  utils::YamlRead<double>(config, "surf_feature_leaf_size",
                          surf_feature_leaf_size, 0.2);

  utils::YamlRead<double>(config, "lidar_min_distance", lidar_min_distance,
                          1.0);
  utils::YamlRead<double>(config, "lidar_max_distance", lidar_max_distance,
                          80.0);

  match_method = StrToMatchMethod(config["match_method"].as<std::string>());

  utils::YamlRead<double>(config, "corner_ds_leaf_size", corner_ds_leaf_size,
                          0.1);
  utils::YamlRead<double>(config, "surface_ds_leaf_size", surface_ds_leaf_size,
                          0.2);
  utils::YamlRead<int>(config, "corner_feature_min_valid_num",
                       corner_feature_min_valid_num, 10);
  utils::YamlRead<int>(config, "surface_feature_min_valid_num",
                       surface_feature_min_valid_num, 100);

  utils::YamlRead<double>(config, "keyframe_search_radius",
                          keyframe_search_radius, 30);
  utils::YamlRead<double>(config, "keyframe_adding_distance_threshold",
                          keyframe_adding_distance_threshold, 1.0);
  utils::YamlRead<double>(config, "keyframe_adding_angle_threshold",
                          keyframe_adding_angle_threshold, 1.0);
  utils::YamlRead<double>(config, "keyframe_mapping_downsample_size",
                          keyframe_mapping_downsample_size, 1.0);

  utils::YamlRead<bool>(config, "auto_loop_closure", auto_loop_closure, false);
  utils::YamlRead<double>(config, "loop_closure_searching_distance",
                          loop_closure_searching_distance, 1.0);
  utils::YamlRead<double>(config, "loop_closure_distance_interval",
                          loop_closure_distance_interval, 1.0);
  utils::YamlRead<double>(config, "loop_closure_score_threshold",
                          loop_closure_score_threshold, 1.0);
  utils::YamlRead<bool>(config, "mono_layer_motion", mono_layer_motion, false);
  utils::YamlRead<double>(config, "initial_window_length",
                          initial_window_length, 0.75);
  utils::YamlRead<double>(config, "imu_excite_threshold", imu_excite_threshold,
                          0.12);
  utils::YamlRead<double>(config, "gravity_norm", gravity_norm, 9.8);
  utils::YamlRead<double>(config, "imu_accel_noise", imu_accel_noise, 0.0096);
  utils::YamlRead<double>(config, "imu_gyro_noise", imu_gyro_noise, 0.0034);
  utils::YamlRead<double>(config, "imu_accel_bias_noise", imu_accel_bias_noise,
                          0.00064);
  utils::YamlRead<double>(config, "imu_gyro_bias_noise", imu_gyro_bias_noise,
                          0.00035);
  utils::YamlRead<double>(config, "map_resolution", map_resolution, 0.1);
}

void LidarImuMappingParams::Print() {
  std::cout << "\033[1;32m----> Lidar IMU Mapping Params <----\033[0m"
            << std::endl;

  PrintLine("num_ring", num_ring);
  PrintLine("num_horizon", num_horizon);
  PrintLine("number_of_cores", number_of_cores);
  PrintLine("edge_threshold", edge_threshold);
  PrintLine("surf_threshold", surf_threshold);
  PrintLine("surf_feature_leaf_size", surf_feature_leaf_size);
  PrintLine("lidar_min_distance", lidar_min_distance);
  PrintLine("lidar_max_distance", lidar_max_distance);
  PrintLine("match_method", MatchMethodToStr(match_method));
  PrintLine("corner_ds_leaf_size", corner_ds_leaf_size);
  PrintLine("surface_ds_leaf_size", surface_ds_leaf_size);
  PrintLine("corner_feature_min_valid_num", corner_feature_min_valid_num);
  PrintLine("surface_feature_min_valid_num", surface_feature_min_valid_num);
  PrintLine("keyframe_search_radius", keyframe_search_radius);
  PrintLine("keyframe_adding_distance_threshold",
            keyframe_adding_distance_threshold);
  PrintLine("keyframe_adding_angle_threshold", keyframe_adding_angle_threshold);
  PrintLine("keyframe_mapping_downsample_size",
            keyframe_mapping_downsample_size);

  PrintLine("auto_loop_closure", auto_loop_closure);
  PrintLine("loop_closure_searching_distance", loop_closure_searching_distance);
  PrintLine("loop_closure_distance_interval", loop_closure_distance_interval);
  PrintLine("loop_closure_score_threshold", loop_closure_score_threshold);
  PrintLine("mono_layer_motion", mono_layer_motion);

  PrintLine("initial_window_length", initial_window_length);
  PrintLine("imu_excite_threshold", imu_excite_threshold);
  PrintLine("gravity_norm", gravity_norm);
  PrintLine("imu_accel_noise", imu_accel_noise);
  PrintLine("imu_gyro_noise", imu_gyro_noise);
  PrintLine("imu_accel_bias_noise", imu_accel_bias_noise);
  PrintLine("imu_gyro_bias_noise", imu_gyro_bias_noise);

  PrintLine("map_resolution", map_resolution);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping