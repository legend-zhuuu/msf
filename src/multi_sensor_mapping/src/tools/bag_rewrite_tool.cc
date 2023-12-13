#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bag_rewrite_tool");

  std::string bag_path =
      "/home/hkw/dataset/Sizu/pandar_imu_2023-09-28-12-08-14.bag";
  std::string output_bag_path = "/home/hkw/dataset/Sizu/pandar_imu.bag";

  std::string lidar_topic = "/hesai/pandar";
  std::string imu_topic = "/imu/data";

  rosbag::Bag bag;
  rosbag::Bag bag_write;
  bag.open(bag_path, rosbag::bagmode::Read);
  bag_write.open(output_bag_path, rosbag::bagmode::Write);

  rosbag::View view;
  std::vector<std::string> topics;

  topics.push_back(lidar_topic);
  topics.push_back(imu_topic);
  view.addQuery(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : view) {
    ros::Time rosbag_time = m.getTime();
    const std::string& topic = m.getTopic();

    if (topic == lidar_topic) {
      sensor_msgs::PointCloud2::Ptr cloud_msg =
          m.instantiate<sensor_msgs::PointCloud2>();
      cloud_msg->header.stamp = rosbag_time;
      bag_write.write(lidar_topic, rosbag_time, cloud_msg);
    } else if (topic == imu_topic) {
      sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
      bag_write.write(imu_topic, rosbag_time, imu_msg);
    }
  }

  bag.close();
  bag_write.close();

  return 0;
}