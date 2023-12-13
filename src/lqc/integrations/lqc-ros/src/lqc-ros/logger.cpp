#include <lqc-ros/logger.h>
#include <std_msgs/Float32MultiArray.h>

namespace lqc {
namespace _ros {

Logger::Logger(const ros::NodeHandle &nh) : nh_(nh) {
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
  joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("twist", 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

  imu_msg_.header.seq = 0;
  joint_msg_.header.seq = 0;
  joint_msg_.position.resize(28, 0.);
  joint_msg_.velocity.resize(28, 0.);
  joint_msg_.effort.resize(28, 0.);
  joint_msg_.name = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      "FR_foot", "FL_foot", "RR_foot", "RL_foot",
      "FR_hip_cmd", "FR_thigh_cmd", "FR_calf_cmd",
      "FL_hip_cmd", "FL_thigh_cmd", "FL_calf_cmd",
      "RR_hip_cmd", "RR_thigh_cmd", "RR_calf_cmd",
      "RL_hip_cmd", "RL_thigh_cmd", "RL_calf_cmd",
  };
}

void Logger::pubProprio(const sdk::LowState &state, const sdk::LowCmd &cmd) {
  auto timestamp = ros::Time::now();
  imu_msg_.header.stamp = timestamp;
  imu_msg_.angular_velocity.x = state.imu.gyroscope[0];
  imu_msg_.angular_velocity.y = state.imu.gyroscope[1];
  imu_msg_.angular_velocity.z = state.imu.gyroscope[2];
  imu_msg_.linear_acceleration.x = state.imu.accelerometer[0];
  imu_msg_.linear_acceleration.y = state.imu.accelerometer[1];
  imu_msg_.linear_acceleration.z = state.imu.accelerometer[2];
  imu_msg_.orientation.w = state.imu.quaternion[0];
  imu_msg_.orientation.x = state.imu.quaternion[1];
  imu_msg_.orientation.y = state.imu.quaternion[2];
  imu_msg_.orientation.z = state.imu.quaternion[3];
  imu_pub_.publish(imu_msg_);
  imu_msg_.header.seq++;

  joint_msg_.header.stamp = timestamp;
  for (int i = 0; i < 12; ++i) {
    const auto &joint = state.motorState[i];
    joint_msg_.position[i] = joint.q;
    joint_msg_.velocity[i] = joint.dq;
    joint_msg_.effort[i] = joint.tauEst;
  }
  for (int i = 0; i < 4; ++i) {
    joint_msg_.position[i + 12] = 0.;
    joint_msg_.velocity[i + 12] = 0.;
    joint_msg_.effort[i + 12] = state.footForce[i];
  }
  for (int i = 0; i < 12; ++i) {
    const auto &joint = state.motorState[i];
    const auto &joint_cmd = cmd.motorCmd[i];
    joint_msg_.position[i + 16] = joint_cmd.q;
    joint_msg_.velocity[i + 16] = joint_cmd.dq;
    joint_msg_.effort[i + 16] = (joint_cmd.q - joint.q) * joint_cmd.Kp
//    joint_msg_.effort[i + 16] = (joint_cmd.q - joint.q) * 60
        + (joint_cmd.dq - joint.dq) * joint_cmd.Kd + joint_cmd.tau;
  }
  joint_pub_.publish(joint_msg_);
  joint_msg_.header.seq++;
}

void Logger::pubArray(const std::string &name, cArrX vec) {
  auto channel = pub_map_.find(name);
  if (channel == pub_map_.end()) {
    pub_map_[name] = nh_.advertise<std_msgs::Float32MultiArray>(name, 1);
    channel = pub_map_.find(name);
  }
  std_msgs::Float32MultiArray msg;
  msg.data.resize(vec.size());
  std::copy(vec.data(), vec.data() + vec.size(), msg.data.data());
  channel->second.publish(msg);
}

void Logger::pubTwist(Vec3 lin_vel, Vec3 ang_vel) {
  twist_msg_.linear.x = lin_vel[0];
  twist_msg_.linear.y = lin_vel[1];
  twist_msg_.linear.z = lin_vel[2];
  twist_msg_.angular.x = ang_vel[0];
  twist_msg_.angular.y = ang_vel[1];
  twist_msg_.angular.z = ang_vel[2];
  twist_pub_.publish(twist_msg_);
}

void Logger::pubOdom(Vec3 pos, Vec4 orn, Vec3 lin_vel, Vec3 ang_vel) {
  odom_msg_.header.stamp = ros::Time::now();
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];
  odom_msg_.pose.pose.orientation.w = orn[0];
  odom_msg_.pose.pose.orientation.x = orn[1];
  odom_msg_.pose.pose.orientation.y = orn[2];
  odom_msg_.pose.pose.orientation.z = orn[3];
  odom_msg_.twist.twist.linear.x = lin_vel[0];
  odom_msg_.twist.twist.linear.y = lin_vel[1];
  odom_msg_.twist.twist.linear.z = lin_vel[2];
  odom_msg_.twist.twist.angular.x = ang_vel[0];
  odom_msg_.twist.twist.angular.y = ang_vel[1];
  odom_msg_.twist.twist.angular.z = ang_vel[2];
  odom_pub_.publish(odom_msg_);
}

}  // namespace _ros
}  // namespace lqc
