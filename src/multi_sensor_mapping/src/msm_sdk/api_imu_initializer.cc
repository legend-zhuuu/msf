#include "msm_sdk/api_imu_initializer.h"

#include "multi_sensor_mapping/frontend/imu/imu_initializer.h"

using msm_sdk::APIImuInitializer;

bool exit_process_flag = false;
bool start_flag = false;
double gravity = 9.81;
double window_lenth = 0.75;
double imu_excite_threshold = 0.12;

multi_sensor_mapping::IMUData ConvertImu(const msm_sdk::IMU& _imu_data) {
  multi_sensor_mapping::IMUData data;
  data.timestamp = _imu_data.timestamp;
  data.gyro = _imu_data.gyro;
  data.accel = _imu_data.accel;
  data.orientation = _imu_data.orientation;
  return data;
}

APIImuInitializer::APIImuInitializer() {}

void APIImuInitializer::Init() {
  imu_initializer_ptr_ = std::make_shared<multi_sensor_mapping::ImuInitializer>(
      gravity, window_lenth, imu_excite_threshold);
}

void APIImuInitializer::Init(double _excite_threshold) {
  imu_initializer_ptr_ = std::make_shared<multi_sensor_mapping::ImuInitializer>(
      gravity, window_lenth, _excite_threshold);
}

void APIImuInitializer::Start() {
  if (start_flag) {
    return;
  }
  exit_process_flag = false;

  process_thread_ = std::thread(std::bind(&APIImuInitializer::Process, this));

  start_flag = true;
}

void APIImuInitializer::Stop() {
  if (!start_flag) {
    return;
  }

  exit_process_flag = true;
  process_thread_.join();

  imu_initializer_ptr_->Clear();

  start_flag = false;
}

void APIImuInitializer::InputIMU(const IMU& _data) {
  multi_sensor_mapping::IMUData imu_data = ConvertImu(_data);
  imu_initializer_ptr_->FeedImuData(imu_data);
}

void APIImuInitializer::RegStationaryCoeffCallback(
    const std::function<void(const bool&, const double&)>& _cb) {
  cb_put_stationary_coeff_ = _cb;
}

void APIImuInitializer::Process() {
  while (!exit_process_flag) {
    double coeff = imu_initializer_ptr_->ComputeImuStationaryCoeff();

    bool imu_static_flag = false;
    if (coeff > 0) {
      if (coeff < imu_excite_threshold) {
        imu_static_flag = true;
      }
    }

    if (cb_put_stationary_coeff_) {
      cb_put_stationary_coeff_(imu_static_flag, coeff);
    }

    usleep(100000);
  }
}
