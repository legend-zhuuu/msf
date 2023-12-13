#include "multi_sensor_mapping/param/pose_graph_params.h"

#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

PoseGraphParams::PoseGraphParams(std::string _name)
    : loop_closure_searching_distance(5.0),
      loop_closure_distance_interval(15.0),
      loop_closure_score_threshold(0.5),
      mono_layer_motion(false),
      map_resolution(0.1),
      output_detail_info(false),
      output_grid_map(false) {
  name_ = _name;
}

void PoseGraphParams::Load(std::string _path_of_yaml) {
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

  utils::YamlRead<double>(config, "loop_closure_searching_distance",
                          loop_closure_searching_distance, 1.0);
  utils::YamlRead<double>(config, "loop_closure_distance_interval",
                          loop_closure_distance_interval, 1.0);
  utils::YamlRead<double>(config, "loop_closure_score_threshold",
                          loop_closure_score_threshold, 1.0);
  utils::YamlRead<bool>(config, "mono_layer_motion", mono_layer_motion, false);

  utils::YamlRead<double>(config, "map_resolution", map_resolution, 0.1);
  utils::YamlRead<bool>(config, "output_detail_info", output_detail_info,
                        false);
  utils::YamlRead<bool>(config, "output_grid_map", output_grid_map, false);
}

void PoseGraphParams::Print() {
  std::cout << "\033[1;32m----> Pose Graph Params <----\033[0m" << std::endl;

  PrintLine("loop_closure_searching_distance", loop_closure_searching_distance);
  PrintLine("loop_closure_distance_interval", loop_closure_distance_interval);
  PrintLine("loop_closure_score_threshold", loop_closure_score_threshold);
  PrintLine("mono_layer_motion", mono_layer_motion);
  PrintLine("map_resolution", map_resolution);
  PrintLine("output_detail_info", output_detail_info);
  PrintLine("output_grid_map", output_grid_map);

  std::cout << "\033[1;32m--------------------------\033[0m" << std::endl;

  std::cout << std::endl;
}

}  // namespace multi_sensor_mapping