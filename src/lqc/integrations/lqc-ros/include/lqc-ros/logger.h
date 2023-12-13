#ifndef LQC_ROS_LOGGER_H_
#define LQC_ROS_LOGGER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <lqc/lqc.h>

namespace lqc {
namespace _ros {

class Logger : public lqc::Logger {
 public:
  explicit Logger(const ros::NodeHandle &nh);
  void pubProprio(const sdk::LowState &, const sdk::LowCmd &) override;
  void pubArray(const std::string &name, cArrX) override;
  void pubTwist(Vec3 lin_vel, Vec3 ang_vel) override;
  void pubOdom(Vec3 pos, Vec4 orn, Vec3 lin_vel, Vec3 ang_vel) override;

 private:
  ros::NodeHandle nh_;
  ros::Publisher joint_pub_, imu_pub_;
  ros::Publisher twist_pub_, odom_pub_;
  sensor_msgs::JointState joint_msg_;
  sensor_msgs::Imu imu_msg_;
  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;

  std::map<std::string, ros::Publisher> pub_map_;
};

}  // namespace _ros
}  // namespace lqc

#endif  // LQC_ROS_LOGGER_H_
