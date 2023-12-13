#include "multi_sensor_mapping/param/param_set.h"

#include "multi_sensor_mapping/param/clins_params.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/param/lidar_detection_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/param_factory.h"
#include "multi_sensor_mapping/param/patchwork_params.h"
#include "multi_sensor_mapping/param/pose_graph_params.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/param/tiny_location_params.h"
#include "multi_sensor_mapping/utils/utils_yaml.h"

namespace multi_sensor_mapping {

MappingMode StrToMapingMode(std::string _mode) {
  if (_mode == "LIM") {
    return MappingMode::LIDAR_IMU_MAPPING;
  } else if (_mode == "LOM") {
    return MappingMode::LIDAR_ODOM_MAPPING;
  } else if (_mode == "PLM") {
    return MappingMode::PURE_LIDAR_MAPPING;
  } else if (_mode == "CLINS") {
    return MappingMode::CLINS_MAPPING;
  }

  return MappingMode::LIDAR_IMU_MAPPING;
}

ParamType StrToParamType(std::string _type) {
  if (_type == "SENSOR") {
    return ParamType::PARAM_TYPE_SENSOR;
  } else if (_type == "EXTRINSIC") {
    return ParamType::PARAM_TYPE_EXTRINSIC;
  } else if (_type == "LIMAPPING") {
    return ParamType::PARAM_TYPE_LIDAR_IMU_MAPPING;
  } else if (_type == "PATCHWORK") {
    return ParamType::PARAM_TYPE_PATCHWORK;
  } else if (_type == "CLINS") {
    return ParamType::PARAM_TYPE_CLINS;
  } else if (_type == "FASTLIO") {
    return ParamType::PARAM_TYPE_FAST_LIO;
  } else if (_type == "POSEGRAPH") {
    return ParamType::PARAM_TYPE_POSE_GRAPH;
  } else if (_type == "TINYLOCATION") {
    return ParamType::PARAM_TYPE_TINY_LOCATION;
  } else if (_type == "DETECTION") {
    return ParamType::PARAM_TYPE_LIDAR_DETECTION;
  }

  return ParamType::PARAM_TYPE_ERROR;
}  // namespace multi_sensor_mapping

ParamSet::ParamSet() {}

bool ParamSet::Load(std::string _path) {
  std::string path_of_yaml = _path + "/param_set.yaml";
  YAML::Node config;
  try {
    config = YAML::LoadFile(path_of_yaml);
  } catch (...) {
    AERROR_F(
        "The format of config file [ %s ] is wrong, Please check (e.g. "
        "indentation).",
        path_of_yaml.c_str());
    return false;
  }

  mapping_mode_ = StrToMapingMode(config["mode"].as<std::string>());
  YAML::Node params_config = utils::YamlSubNodeAbort(config, "params");

  for (size_t i = 0; i < params_config.size(); i++) {
    ParamType param_type =
        StrToParamType(params_config[i]["type"].as<std::string>());
    if (param_type == ParamType::PARAM_TYPE_ERROR) {
      AWARN_F("[ParamSet] Param type wrong , Param type : %s ",
              params_config[i]["type"].as<std::string>().c_str());
    }
    std::string param_path =
        _path + "/" + params_config[i]["file"].as<std::string>();

    auto new_param = ParamFactory::GetInstance()->NewParam(param_type);
    new_param->Load(param_path);

    params_.insert(std::make_pair(param_type, new_param));
  }

  return true;
}

void ParamSet::PrintAll() {
  std::map<ParamType, std::shared_ptr<ParamBase>>::iterator iter;
  for (iter = params_.begin(); iter != params_.end(); iter++) {
    iter->second->Print();
  }
}

std::shared_ptr<SensorParams> ParamSet::GetSensorParams() {
  std::shared_ptr<SensorParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_SENSOR);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<SensorParams>(iter->second);
  }
  return output_param;
}

std::shared_ptr<ExtrinsicParams> ParamSet::GetExtrinsicParams() {
  std::shared_ptr<ExtrinsicParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_EXTRINSIC);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<ExtrinsicParams>(iter->second);
  }
  return output_param;
}

std::shared_ptr<LidarImuMappingParams> ParamSet::GetLidarImuMappingParams() {
  std::shared_ptr<LidarImuMappingParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_LIDAR_IMU_MAPPING);
  if (iter != params_.end()) {
    output_param =
        std::dynamic_pointer_cast<LidarImuMappingParams>(iter->second);
  }
  return output_param;
}

std::shared_ptr<PatchWorkParams> ParamSet::GetPatchworkParams() {
  std::shared_ptr<PatchWorkParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_PATCHWORK);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<PatchWorkParams>(iter->second);
  }
  return output_param;
}

std::shared_ptr<ClinsParams> ParamSet::GetClinsParams() {
  std::shared_ptr<ClinsParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_CLINS);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<ClinsParams>(iter->second);
  }

  return output_param;
}

std::shared_ptr<FastLIOParams> ParamSet::GetFastLIOParams() {
  std::shared_ptr<FastLIOParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_FAST_LIO);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<FastLIOParams>(iter->second);
  }

  return output_param;
}

std::shared_ptr<PoseGraphParams> ParamSet::GetPoseGraphParams() {
  std::shared_ptr<PoseGraphParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_POSE_GRAPH);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<PoseGraphParams>(iter->second);
  }

  return output_param;
}

std::shared_ptr<TinyLocationParams> ParamSet::GetTinyLocationParams() {
  std::shared_ptr<TinyLocationParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_TINY_LOCATION);
  if (iter != params_.end()) {
    output_param = std::dynamic_pointer_cast<TinyLocationParams>(iter->second);
  }

  return output_param;
}

std::shared_ptr<LidarDetectionParams> ParamSet::GetLidarDetectionParams() {
  std::shared_ptr<LidarDetectionParams> output_param;
  auto iter = params_.find(ParamType::PARAM_TYPE_LIDAR_DETECTION);
  if (iter != params_.end()) {
    output_param =
        std::dynamic_pointer_cast<LidarDetectionParams>(iter->second);
  }

  return output_param;
}

}  // namespace multi_sensor_mapping