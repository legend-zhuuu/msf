//
// Created by zdy on 23-10-16.
//
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <deque>

#define _dt 1/400

Eigen::Vector3d _gravity(0., 0., -9.8066);
Eigen::Vector3f current_linvel(0., 0., 0.);
Eigen::Vector3f current_angvel(0., 0., 0.);
Eigen::Vector3f current_pose(0., 0., 0.);
Eigen::Quaterniond current_orientation(0., 0., 0., 1.);
Eigen::Vector3f next_linvel(0., 0., 0.);
Eigen::Vector3f next_angvel(0., 0., 0.);
Eigen::Vector3f next_pose(0., 0., 0.);
Eigen::Quaterniond next_orientation(0., 0., 0., 1.);
std::deque<sensor_msgs::Imu> imuQueOpt;
tf::StampedTransform robotBase2Map;
Eigen::Vector3f est_vel_world_based(0.f, 0.f, 0.f);


class SensorDataProcessor {
public:
    SensorDataProcessor(const ros::NodeHandle &nh) : nh_(nh) {
        nh_.param<std::string>("imu_topic", imu_topic_, "/imu/data");
        lidar_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/lidar_pose", 10,
                                                                    &SensorDataProcessor::LidarPoseCallback, this);
        odom_velocity_sub_ = nh_.subscribe<nav_msgs::Odometry>("/estimate_velocity", 10,
                                                               &SensorDataProcessor::OdomVelCallback, this);
//        est_vel_sub_ = nh_.subscribe<aliengo_s2r::PolicyState>("/aliengo/policy", 10, &SensorDataProcessor::EstVelCallback,
//                                                   this);
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 100, &SensorDataProcessor::ImuCallback, this);
        robor_base_pose_inter_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_base_pose_inter", 1, true);
    }

    void LidarPoseCallback(const geometry_msgs::PoseStampedConstPtr &lidar_pose);

    void OdomVelCallback(const nav_msgs::OdometryConstPtr &velocity_data);

//    void EstVelCallback(const aliengo_s2r::PolicyState::ConstPtr &est_vel_msg);

    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);

private:
    ros::NodeHandle nh_;
    std::string imu_topic_;
    // prepare subscriber and publisher
    tf::TransformListener tf_listener_;
    ros::Publisher robor_base_pose_inter_pub_;
    ros::Subscriber lidar_pose_sub_;
    ros::Subscriber odom_velocity_sub_;
    ros::Subscriber est_vel_sub_;
    ros::Subscriber imu_sub_;
};


void SensorDataProcessor::LidarPoseCallback(const geometry_msgs::PoseStampedConstPtr &lidar_pose) {
    // update robot pose when lidar pose updated
    try {
        tf_listener_.waitForTransform("/map", "/base", ros::Time(0), ros::Duration(5));
        tf_listener_.lookupTransform("/map", "/base", ros::Time(0), robotBase2Map);
        current_pose.x() = robotBase2Map.getOrigin().x();
        current_pose.y() = robotBase2Map.getOrigin().y();
        current_pose.z() = robotBase2Map.getOrigin().z();
        current_orientation.x() = robotBase2Map.getRotation().x();
        current_orientation.y() = robotBase2Map.getRotation().y();
        current_orientation.z() = robotBase2Map.getRotation().z();
        current_orientation.w() = robotBase2Map.getRotation().w();
//        std::cout<<"pose: "<<lidar_pose->pose.position<<"#####"<<current_pose<<std::endl;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("transfrom exception : %s", ex.what());
        return;
    }
}

//void SensorDataProcessor::EstVelCallback(const aliengo_s2r::PolicyState::ConstPtr &est_vel_msg){
//    Eigen::Vector3f est_vel_robot_based(est_vel_msg->velest[0], est_vel_msg->velest[1], est_vel_msg->velest[2]);
//    try{
//        tf_listener_.waitForTransform("/map", "/base", ros::Time(0), ros::Duration(10.0));
//        tf_listener_.lookupTransform("/map", "/base", ros::Time(0), robotBase2Map);
//        Eigen::Translation3f tl_robot2map(robotBase2Map.getOrigin().getX(), robotBase2Map.getOrigin().getY(), robotBase2Map.getOrigin().getZ());
//        double roll, pitch, yaw;
//        Eigen::Matrix3Xf tf_robot2map;
//        tf::Matrix3x3(robotBase2Map.getRotation()).getEulerYPR(yaw, pitch, roll);
//        Eigen::AngleAxisf rot_x_robot2map(roll, Eigen::Vector3f::UnitX());
//        Eigen::AngleAxisf rot_y_robot2map(pitch, Eigen::Vector3f::UnitY());
//        Eigen::AngleAxisf rot_z_robot2map(yaw, Eigen::Vector3f::UnitZ());
//        tf_robot2map = (rot_z_robot2map * rot_y_robot2map * rot_x_robot2map).matrix();
//        est_vel_world_based = tf_robot2map * est_vel_robot_based;
////        std::cout<<"robot_base_velx: "<<est_vel_robot_based.x()<<"\n"<<"world_base_velx: "<<est_vel_world_based.x()<<std::endl;
//    }
//    catch (tf::TransformException ex) {
//        ROS_WARN("transfrom exception : %s", ex.what());
//        return;
//    }
//}

void SensorDataProcessor::OdomVelCallback(const nav_msgs::OdometryConstPtr &velocity_data) {
    // get velocity from fast_lio
    current_linvel.x() = velocity_data->twist.twist.linear.x;
    current_linvel.y() = velocity_data->twist.twist.linear.y;
    current_linvel.z() = velocity_data->twist.twist.linear.z;
    current_angvel.x() = velocity_data->twist.twist.angular.x;
    current_angvel.y() = velocity_data->twist.twist.angular.y;
    current_angvel.z() = velocity_data->twist.twist.angular.z;
    ROS_INFO_THROTTLE(
            5.0,
            "Current vel pose is (%f, %f, %f).",
            velocity_data->twist.twist.linear.x, velocity_data->twist.twist.linear.y,
            velocity_data->twist.twist.linear.z);
}

Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_x;
}

Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w) {
    Eigen::Matrix<double, 4, 4> mat;
    mat.block(0, 0, 3, 3) = -skew_x(w);
    mat.block(3, 0, 1, 3) = -w.transpose();
    mat.block(0, 3, 3, 1) = w;
    mat(3, 3) = 0;
    return mat;
}

Eigen::Matrix<double, 4, 1> quatnorm(Eigen::Matrix<double, 4, 1> q_t) {
    if (q_t(3, 0) < 0) {
        q_t *= -1;
    }
    return q_t / q_t.norm();
}

void predict_mean_discrete(const Eigen::Quaterniond &q, const Eigen::Vector3d &v, const Eigen::Vector3d p,
                           double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                           Eigen::Quaterniond &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p) {
    // If we are averaging the IMU, then do so
    Eigen::Vector3d w_hat = w_hat1;
    Eigen::Vector3d a_hat = a_hat1;
    // Pre-compute things
    double w_norm = w_hat.norm();
    Eigen::Matrix<double, 4, 4> I_4x4 = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 3, 3> R_Gtoi = q.matrix();
    // Orientation: Equation (101) and (103) and of Trawny indirect TR
    Eigen::Matrix<double, 4, 4> bigO;
    if (w_norm > 1e-20) {
        bigO = cos(0.5 * w_norm * dt) * I_4x4 + 1 / w_norm * sin(0.5 * w_norm * dt) * Omega(w_hat);
    } else {
        bigO = I_4x4 + 0.5 * dt * Omega(w_hat);
    }
    Eigen::Vector4d eq(q.w(), q.x(), q.y(), q.z());
    new_q = quatnorm(bigO * eq);
    // new_q = rot_2_quat(exp_so3(-w_hat*dt)*R_Gtoi);
    // Velocity: just the acceleration in the local frame, minus global gravity
    new_v = v + R_Gtoi.transpose() * a_hat * dt - _gravity * dt;
    // Position: just velocity times dt, with the acceleration integrated twice
    new_p = p + v * dt + 0.5 * R_Gtoi.transpose() * a_hat * dt * dt - 0.5 * _gravity * dt * dt;
}

void get_imu_data(sensor_msgs::Imu imu_data, Eigen::Quaterniond &q, Eigen::Vector3d &ang_v, Eigen::Vector3d &acc) {
    q.x() = imu_data.orientation.x;
    q.y() = imu_data.orientation.y;
    q.z() = imu_data.orientation.z;
    q.w() = imu_data.orientation.w;
    ang_v.x() = imu_data.angular_velocity.x;
    ang_v.y() = imu_data.angular_velocity.y;
    ang_v.z() = imu_data.angular_velocity.z;
    acc.x() = imu_data.linear_acceleration.x;
    acc.y() = imu_data.linear_acceleration.y;
    acc.z() = imu_data.linear_acceleration.z;
}


void SensorDataProcessor::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    // get high-rate imu data to interpolate pose.
    imuQueOpt.push_back(*imu_msg);

    // if the size of imuQueOpt is 10(50Hz), integrate the imu data.
    if (imuQueOpt.size() == 8) {
        while (!imuQueOpt.empty()) {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu thisImu = imuQueOpt.front();
//            Eigen::Quaterniond imu_q;
//            Eigen::Vector3d imu_angvel;
//            Eigen::Vector3d imu_acc;
//            get_imu_data(thisImu, imu_q, imu_angvel, imu_acc);
//            //currentCorrectionTime是激光里程计时间数据，delta_t定义时赋值0，没找到修改此参数值的代码
//            predict_mean_discrete(imu_q, current_linvel, current_pose,
//                                  _dt, imu_angvel, imu_acc,
//                                  next_orientation, next_linvel, next_pose);
            imuQueOpt.pop_front();
//            current_linvel = next_linvel;
//            current_pose = next_pose;
//            current_orientation = next_orientation;
        }
        geometry_msgs::PoseStamped pose_lidar, pose_base;
        // get the transform between map and robot base
        try {
            current_pose = current_pose + est_vel_world_based * 0.02;
            pose_base.header = imu_msg->header;
            pose_base.header.frame_id = "/map";
            pose_base.pose.position.x = current_pose.x();
            pose_base.pose.position.y = current_pose.y();
            pose_base.pose.position.z = current_pose.z();
            pose_base.pose.orientation.x = current_orientation.x();
            pose_base.pose.orientation.y = current_orientation.y();
            pose_base.pose.orientation.z = current_orientation.z();
            pose_base.pose.orientation.w = current_orientation.w();
            robor_base_pose_inter_pub_.publish(pose_base);
//            std::cout<<pose_base.pose<<std::endl;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("transfrom exception : %s", ex.what());
            return;
        }
    }
}

int main(int argc, char **argv) {
    // Initialize node and publisher.
    ros::init(argc, argv, "lidar_pose_inter");
    ros::NodeHandle nh("~");

    SensorDataProcessor processor_(nh);
    // Work with grid map in a loop.
    ros::Rate rate(1000.0);

    while (nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

