#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include "multi_sensor_mapping/frontend/lidar/reflection_column_extractor.h"
#include "multi_sensor_mapping/utils/data_converter.h"
#include "multi_sensor_mapping/utils/utils_pointcloud.h"
#include "multi_sensor_mapping/visualizer/lidar_mapper_ros_visualizer.h"

using namespace multi_sensor_mapping;

int main(int argc, char** argv) {
  ros::init(argc, argv, "reflection_column_viewer");
  ros::NodeHandle nh_param("~");
  ros::NodeHandle nh;

  ros::Publisher pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1);

  LidarMapperRosVisualizer lidar_mapper_ros_visualizer;

  std::string bag_path;
  std::string lidar_topic;
  nh_param.param<std::string>("bag_path", bag_path, "");
  nh_param.param<std::string>("lidar_topic", lidar_topic, "/rslidar_points");

  std::shared_ptr<DataConverter> data_converter_ =
      std::make_shared<DataConverter>();

  float interest_radius = 5.0;
  float column_diamater = 0.05;
  float column_height = 1;

  std::shared_ptr<ReflectionColumnExtractor> reflection_column_extractor =
      std::make_shared<ReflectionColumnExtractor>(interest_radius,
                                                  column_diamater);

  // 读取数据包
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  rosbag::View view;
  std::vector<std::string> topics;

  topics.push_back(lidar_topic);

  rosbag::View view_full;
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  ros::Time time_finish = view_full.getEndTime();
  ros::Duration delta_durr = ros::Duration(0.1);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init - delta_durr,
                time_finish + delta_durr);

  TimedCloudData cloud_data;
  bool cloud_ready_flag = false;

  for (rosbag::MessageInstance const m : view) {
    const std::string& topic = m.getTopic();
    if (lidar_topic == topic) {
      std::string data_type = m.getDataType();
      if (data_type == std::string("sensor_msgs/PointCloud2")) {
        sensor_msgs::PointCloud2::ConstPtr lidar_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        cloud_data = data_converter_->ConvertRsCloudData(lidar_msg);
        cloud_ready_flag = true;
      }
    }

    if (!cloud_ready_flag) {
      continue;
    }

    // 反光柱提取
    CloudTypePtr cloud(new CloudType);
    pcl::copyPointCloud(*cloud_data.cloud, *cloud);
    std::vector<PointType> columns_center =
        reflection_column_extractor->ExtractColumns(cloud);
    if (!columns_center.empty()) {
      std::vector<ReflectionColumn> reflection_columns;
      for (size_t i = 0; i < columns_center.size(); i++) {
        ReflectionColumn reflection_column;
        reflection_column.id = (int)i;
        reflection_column.position =
            Eigen::Vector3d(columns_center[i].x, columns_center[i].y, 0);
        reflection_column.diamater = column_diamater;
        reflection_column.length = column_height;
        reflection_columns.push_back(reflection_column);
      }

      lidar_mapper_ros_visualizer.DisplayReflectionColumn(reflection_columns);
    }

    // 可视化点云
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_data.cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    pub_cloud.publish(cloud_msg);

    cloud_ready_flag = false;

    std::cout << "Process Cloud , colunm size : " << columns_center.size()
              << std::endl;

    std::getchar();

    if (!ros::ok()) {
      break;
    }
  }
}