#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "msm_sdk/msm_sdk.h"
#include "multi_sensor_mapping/utils/utils_tictoc.h"

using namespace msm_sdk;

APIImuInitializer initializer;

// IMU数据回调函数
void ImuHandler(const sensor_msgs::Imu::ConstPtr& _imu_msg) {
  IMU data;
  data.timestamp = _imu_msg->header.stamp.toSec();
  data.gyro = Eigen::Vector3d(_imu_msg->angular_velocity.x,
                              _imu_msg->angular_velocity.y,
                              _imu_msg->angular_velocity.z);
  data.accel = Eigen::Vector3d(_imu_msg->linear_acceleration.x,
                               _imu_msg->linear_acceleration.y,
                               _imu_msg->linear_acceleration.z);
  data.orientation =
      Eigen::Quaterniond(_imu_msg->orientation.w, _imu_msg->orientation.x,
                         _imu_msg->orientation.y, _imu_msg->orientation.z);
  initializer.InputIMU(data);
}

// 回调函数
void PutImuStationaryCoef(const bool& _static_flag, const double& _coeff) {
  std::cout << "Static flag " << _static_flag << " ,  Coeff " << _coeff
            << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_imu_initializer");
  ros::NodeHandle nh;
  std::string imu_topic = "/imu";

  auto msm_version = msm_sdk::GetVersion();
  std::cout << "MSM SDK Version : " << msm_version.toString() << std::endl;

  initializer.Init();
  // 注册回调函数
  initializer.RegStationaryCoeffCallback(std::bind(
      PutImuStationaryCoef, std::placeholders::_1, std::placeholders::_2));

  ros::Subscriber sub_imu_ =
      nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, ImuHandler);

  initializer.Start();

  ros::spin();
  TicToc timer;
  initializer.Stop();
  std::cout << "initializer stop spend " << timer.toc() << std::endl;

  return 0;
}