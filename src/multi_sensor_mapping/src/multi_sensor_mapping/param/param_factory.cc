#include "multi_sensor_mapping/param/param_factory.h"

#include "multi_sensor_mapping/param/clins_params.h"
#include "multi_sensor_mapping/param/extrinsic_params.h"
#include "multi_sensor_mapping/param/fast_lio_params.h"
#include "multi_sensor_mapping/param/lidar_detection_params.h"
#include "multi_sensor_mapping/param/lidar_imu_mapping_params.h"
#include "multi_sensor_mapping/param/patchwork_params.h"
#include "multi_sensor_mapping/param/pose_graph_params.h"
#include "multi_sensor_mapping/param/sensor_params.h"
#include "multi_sensor_mapping/param/tiny_location_params.h"

namespace multi_sensor_mapping {

std::shared_ptr<ParamFactory> ParamFactory::param_generation_;

std::shared_ptr<ParamFactory> ParamFactory::GetInstance() {
  if (param_generation_.get() == 0) {
    param_generation_.reset(new ParamFactory);
  }
  return param_generation_;
}

ParamBase::Ptr ParamFactory::NewParam(ParamType _param_type) {
  switch (_param_type) {
    case ParamType::PARAM_TYPE_SENSOR: {
      SensorParams::Ptr output(new SensorParams());
      return output;
    }
    case ParamType::PARAM_TYPE_EXTRINSIC: {
      ExtrinsicParams::Ptr output(new ExtrinsicParams());
      return output;
    }
    case ParamType::PARAM_TYPE_LIDAR_IMU_MAPPING: {
      LidarImuMappingParams::Ptr output(new LidarImuMappingParams);
      return output;
    }
    case ParamType::PARAM_TYPE_PATCHWORK: {
      PatchWorkParams::Ptr output(new PatchWorkParams);
      return output;
    }
    case ParamType::PARAM_TYPE_CLINS: {
      ClinsParams::Ptr output(new ClinsParams);
      return output;
    }
    case ParamType::PARAM_TYPE_FAST_LIO: {
      FastLIOParams::Ptr output(new FastLIOParams);
      return output;
    }
    case ParamType::PARAM_TYPE_POSE_GRAPH: {
      PoseGraphParams::Ptr output(new PoseGraphParams);
      return output;
    }
    case ParamType::PARAM_TYPE_TINY_LOCATION: {
      TinyLocationParams::Ptr output(new TinyLocationParams);
      return output;
    }
    case ParamType::PARAM_TYPE_LIDAR_DETECTION: {
      LidarDetectionParams::Ptr output(new LidarDetectionParams);
      return output;
    }

    default:
      break;
  }
  return nullptr;
}

ParamBase::Ptr ParamFactory::NewParam(ParamType _param_type,
                                      std::string _name) {
  switch (_param_type) {
    case ParamType::PARAM_TYPE_SENSOR: {
      SensorParams::Ptr output(new SensorParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_EXTRINSIC: {
      ExtrinsicParams::Ptr output(new ExtrinsicParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_LIDAR_IMU_MAPPING: {
      LidarImuMappingParams::Ptr output(new LidarImuMappingParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_PATCHWORK: {
      PatchWorkParams::Ptr output(new PatchWorkParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_CLINS: {
      ClinsParams::Ptr output(new ClinsParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_FAST_LIO: {
      FastLIOParams::Ptr output(new FastLIOParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_POSE_GRAPH: {
      PoseGraphParams::Ptr output(new PoseGraphParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_TINY_LOCATION: {
      TinyLocationParams::Ptr output(new TinyLocationParams(_name));
      return output;
    }
    case ParamType::PARAM_TYPE_LIDAR_DETECTION: {
      LidarDetectionParams::Ptr output(new LidarDetectionParams(_name));
      return output;
    }

    default:
      break;
  }
  return nullptr;
}

}  // namespace multi_sensor_mapping
