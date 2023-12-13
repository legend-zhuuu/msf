#include "multi_sensor_mapping/frontend/gnss/gnss_converter.h"

#include "multi_sensor_mapping/utils/utils_log.h"

namespace multi_sensor_mapping {

GNSSConverter* GNSSConverter::instance_ = NULL;

GNSSConverter* GNSSConverter::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new GNSSConverter();
  }
  return instance_;
}

GNSSConverter::GNSSConverter()
    : origin_position_initialized_flag_(false),
      origin_longitude_(0),
      origin_latitude_(0),
      origin_altitude_(0) {}

void GNSSConverter::InitOriginPosition(GNSSData& _gnss_data) {
  InitOriginPosition(_gnss_data.lla(0), _gnss_data.lla(1), _gnss_data.lla(2));
}

void GNSSConverter::InitOriginPosition(const double& lat, const double& lon,
                                       const double& alt) {
  origin_longitude_ = lon;
  origin_latitude_ = lat;
  origin_altitude_ = alt;

  geographic_converter_.Reset(origin_latitude_, origin_longitude_,
                              origin_altitude_);

  origin_position_initialized_flag_ = true;
}

void GNSSConverter::UpdateForward(const double& lat, const double& lon,
                                  const double& alt, double& local_E,
                                  double& local_N, double& local_U) {
  if (!origin_position_initialized_flag_) {
    AWARN_F("[GNSSConverter] geographic converter is NOT initialized.");
    return;
  }

  geographic_converter_.Forward(lat, lon, alt, local_E, local_N, local_U);
}

void GNSSConverter::UpdateForward(GNSSData& _gnss_data) {
  UpdateForward(_gnss_data.lla(0), _gnss_data.lla(1), _gnss_data.lla(2),
                _gnss_data.lenu(0), _gnss_data.lenu(1), _gnss_data.lenu(2));
}

void GNSSConverter::UpdateReverse(const double& local_E, const double& local_N,
                                  const double& local_U, double& lat,
                                  double& lon, double& alt) {
  if (!origin_position_initialized_flag_) {
    AWARN_F("[GNSSConverter] geographic converter is NOT initialized.");
    return;
  }
  geographic_converter_.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

void GNSSConverter::UpdateReverse(GNSSData& _gnss_data) {
  UpdateReverse(_gnss_data.lenu(0), _gnss_data.lenu(1), _gnss_data.lenu(2),
                _gnss_data.lla(0), _gnss_data.lla(1), _gnss_data.lla(2));
}

}  // namespace multi_sensor_mapping
