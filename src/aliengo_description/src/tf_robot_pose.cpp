//
// Created by zdy on 23-10-24.
//

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

class Odom2Tf {
public:
    Odom2Tf(const ros::NodeHandle &nh, const std::string &topic) : nh_(nh) {
        lidar_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
                topic, 1, &Odom2Tf::callback, this);
    }

    void callback(const geometry_msgs::PoseStampedConstPtr &lidar_pose_msg) {
        // message declarations
        odom_.header.frame_id = "map";
        odom_.child_frame_id = "base";
        // update transform
        odom_.header.stamp = lidar_pose_msg->header.stamp;
        odom_.transform.translation.x = lidar_pose_msg->pose.position.x;
        odom_.transform.translation.y = lidar_pose_msg->pose.position.y;
        odom_.transform.translation.z = lidar_pose_msg->pose.position.z;
        odom_.transform.rotation.x = lidar_pose_msg->pose.orientation.x;
        odom_.transform.rotation.y = lidar_pose_msg->pose.orientation.y;
        odom_.transform.rotation.z = lidar_pose_msg->pose.orientation.z;
        odom_.transform.rotation.w = lidar_pose_msg->pose.orientation.w;
        br_.sendTransform(odom_);
    }

private:
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped odom_;
    tf::TransformBroadcaster br_;
    ros::Subscriber lidar_pose_sub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_robot_pose");
    ros::NodeHandle nh("~");
    Odom2Tf odom2tf(nh, "/lidar_pose");
    ros::spin();
    return 0;
}
