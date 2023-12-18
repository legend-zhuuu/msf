#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include "detection/elevationSample.h"

#include<time.h>

typedef pcl::PointCloud<pcl::PointXYZ> VPointCloud;
#define width 71
#define length 71
#define res 0.02
// using namespace grid_map;
// Create grid map.
grid_map::GridMap map({"elevation"});
grid_map::GridMap map_inpainted({"elevation_inpainted"});
pcl::PointCloud<pcl::PointXYZ>::Ptr current_assembled_map(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher grid_pub;
ros::Publisher elevation_pub;
ros::Publisher elevation_pc_pub;
ros::Subscriber point_sub;
ros::Subscriber position_sub;
ros::Subscriber map_inpainted_sub;
tf::TransformListener *tf_listener_ptr;  // 此类型的变量不可以定义为全局变量

std::string point_sub_topic;
std::string position_sub_topic;
std::string grid_pub_topic;
std::string elevation_pub_topic;
std::string elevation_pointCloud_pub_topic;
std::string grid_frame_id;
double grid_map_size_x;
double grid_map_size_y;
double grid_map_position_x;
double grid_map_position_y;
double grid_map_resolution;
bool point_update;
bool position_update;
std::vector<double> x(length), y(width);
std::vector<std::vector<double>> X_grid(length, std::vector<double>(width));
std::vector<std::vector<double>> Y_grid(length, std::vector<double>(width));
// void Point2grid_with_elevation(const sensor_msgs::PointCloud2ConstPtr&
// receivePoint)
//{
//    ros::Time timestamp = ros::Time::now();
//
//    sensor_msgs::PointCloud2 publishPoint;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new
//    pcl::PointCloud<pcl::PointXYZ>); pcl::fromROSMsg(*receivePoint,
//    *pointCloud);
//
//    Eigen::VectorXf pointCloudVariances;
//    if (pointCloud->size() != pointCloudVariances.size()) {
//        ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances
//        (%i) do not agree.",
//                  (int) pointCloud->size(), (int) pointCloudVariances.size());
//    }
//
////    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
//    for (unsigned int i = 0; i < pointCloud->size(); ++i){
//        auto& point = pointCloud->points[i];
//        grid_map::Index index;
//        grid_map::Position position(point.x, point.y);
//
//        if (!map.getIndex(position, index)) continue; // Skip this point if it
//        does not lie within the elevation map.
//
//        auto& elevation = map.at("elevation", index);
//        auto& variance = map.at("variance", index);
//        auto& horizontalVarianceX = map.at("horizontal_variance_x", index);
//        auto& horizontalVarianceY = map.at("horizontal_variance_y", index);
//        auto& horizontalVarianceXY = map.at("horizontal_variance_xy", index);
//        auto& color = map.at("color", index);
//        auto& time = map.at("time", index);
//        auto& lowestScanPoint = map.at("lowest_scan_point", index);
//        auto& sensorXatLowestScan = map.at("sensor_x_at_lowest_scan", index);
//        auto& sensorYatLowestScan = map.at("sensor_y_at_lowest_scan", index);
//        auto& sensorZatLowestScan = map.at("sensor_z_at_lowest_scan", index);
//
//        const float& pointVariance = pointCloudVariances(i);
//        const float scanTimeSinceInitialization = (timestamp -
//        initialTime_).toSec();
//
//        if (!map.isValid(index)) {
//            // No prior information in elevation map, use measurement.
//            elevation = point.z;variance = pointVariance;
//            horizontalVarianceX = minHorizontalVariance_;
//            horizontalVarianceY = minHorizontalVariance_;
//            horizontalVarianceXY = 0.0;
//            colorVectorToValue(point.getRGBVector3i(), color);
//            continue;
//        }
//
//        // Deal with multiple heights in one cell.
//        const double mahalanobisDistance = fabs(point.z - elevation) /
//        sqrt(variance); if (mahalanobisDistance >
//        mahalanobisDistanceThreshold_) {
//            if (scanTimeSinceInitialization - time <= scanningDuration_ &&
//            elevation > point.z) {
//                // Ignore point if measurement is from the same point cloud
//                (time comparison) and
//                // if measurement is lower then the elevation in the map.
//            } else if (scanTimeSinceInitialization - time <=
//            scanningDuration_) {
//                // If point is higher.
//                elevation = point.z;
//                variance = pointVariance;
//            } else {
//                variance += multiHeightNoise_;
//            }
//            continue;
//        }
//
//        // Store lowest points from scan for visibility checking.
//        const float pointHeightPlusUncertainty = point.z + 3.0 *
//        sqrt(pointVariance); // 3 sigma. if (std::isnan(lowestScanPoint) ||
//        pointHeightPlusUncertainty < lowestScanPoint){
//            lowestScanPoint = pointHeightPlusUncertainty;
//            const Position3
//            sensorTranslation(transformationSensorToMap.translation());
//            sensorXatLowestScan = sensorTranslation.x();
//            sensorYatLowestScan = sensorTranslation.y();
//            sensorZatLowestScan = sensorTranslation.z();
//        }
//
//        // Fuse measurement with elevation map data.
//        elevation = (variance * point.z + pointVariance * elevation) /
//        (variance + pointVariance); variance = (pointVariance * variance) /
//        (pointVariance + variance);
//        // TODO Add color fusion.
//        colorVectorToValue(point.getRGBVector3i(), color);
//        timestamp = scanTimeSinceInitialization;
//
//        // Horizontal variances are reset.
//        horizontalVarianceX = minHorizontalVariance_;
//        horizontalVarianceY = minHorizontalVariance_;
//        horizontalVarianceXY = 0.0;
//    }
//
//    // Publish grid map.
//    map.setTimestamp(timestamp.toNSec());
//    grid_map_msgs::GridMap message;
//    grid_map::GridMapRosConverter::toMessage(map, message);
//    grid_pub.publish(message);
//    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
//    message.info.header.stamp.toSec());
//
//}

void range_remove(VPointCloud::Ptr &in_cloud_ptr,
                  VPointCloud::Ptr &out_cloud_ptr,
                  const std::string &axis,
                  const std::string &op,
                  double threshold = 100.0) {
    pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZ>());
    pcl::ComparisonOps::CompareOp oper;
    if (op == "<")
        oper = pcl::ComparisonOps::LT;
    else
        oper = pcl::ComparisonOps::GT;

    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>(axis, oper, threshold))
    );
    // Build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(in_cloud_ptr);
    condrem.filter(*out_cloud_ptr);
}

void Point2grid(const sensor_msgs::PointCloud2ConstPtr &receivePoint) {
    pcl::fromROSMsg(*receivePoint, *current_assembled_map);

    if (false) {
        ros::Time timestamp = receivePoint->header.stamp;

        sensor_msgs::PointCloud2 publishPoint;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(
                new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*receivePoint, *pointCloud);

        VPointCloud::Ptr point_cloud_filtered(new VPointCloud());
        point_cloud_filtered = pointCloud;
        //  range_remove(pointCloud, point_cloud_filtered, 2.0);
        //    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
        for (unsigned int i = 0; i < point_cloud_filtered->size(); ++i) {
            auto &point = point_cloud_filtered->points[i];
            grid_map::Index index;
            grid_map::Position position(point.x, point.y);

            if (!map.getIndex(position, index))
                continue;  // Skip this point if it does not lie within the elevation map.

            auto &elevation = map.at("elevation", index);

            if (!map.isValid(index) || point_update) {
                // No prior information in elevation map or should update map in real
                // time, use measurement.
                elevation = point.z;
                continue;
            }
        }
        std::cout << point_cloud_filtered->size() << std::endl;

        // Publish grid map.
        map.setTimestamp(timestamp.toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map, message);
        grid_pub.publish(message);
//    std::cout << "pubed" << std::endl;
        ROS_INFO_THROTTLE(10.0, "Grid map (timestamp %f) published.",
                          message.info.header.stamp.toSec());
    }

}


void MapPositionUpdate(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    // Move map with the center of robot's base position
    // Pose's frame_id should be same with map's frame_id

    // 1. filter point cloud
    clock_t time1 = clock();
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.header = pose->header;
    pose_stamped.pose = pose->pose;
    Eigen::Vector3d robot_position(pose_stamped.pose.pose.position.x, pose_stamped.pose.pose.position.y,
                                   pose_stamped.pose.pose.position.z);
    Eigen::Quaterniond robot_orientation(pose_stamped.pose.pose.orientation.w, pose_stamped.pose.pose.orientation.x,
                                         pose_stamped.pose.pose.orientation.y, pose_stamped.pose.pose.orientation.z);
    double range = 1.0;
    double robot_x_max = robot_position.x() + range;
    double robot_x_min = robot_position.x() - range;
    double robot_y_max = robot_position.y() + range;
    double robot_y_min = robot_position.y() - range;
    double robot_z_max = robot_position.z() + 0.7;
    double robot_z_min = robot_position.z() - 1.0;
//    std::cout << "filter before:" << current_assembled_map->size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_assembled_pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    range_remove(current_assembled_map, filtered_assembled_pointCloud, "x", "<", robot_x_max);
//    range_remove(filtered_assembled_pointCloud, filtered_assembled_pointCloud, "x", ">", robot_x_min);
//    range_remove(filtered_assembled_pointCloud, filtered_assembled_pointCloud, "y", "<", robot_y_max);
//    range_remove(filtered_assembled_pointCloud, filtered_assembled_pointCloud, "y", ">", robot_y_min);
//    range_remove(filtered_assembled_pointCloud, filtered_assembled_pointCloud, "z", "<", robot_z_max);
//    range_remove(filtered_assembled_pointCloud, filtered_assembled_pointCloud, "z", ">", robot_z_min);
    // this filter is much faster than before.
    pcl::PassThrough<pcl::PointXYZ> pass;
    // x
    pass.setInputCloud(current_assembled_map);          // 1. 设置输入源
    pass.setFilterFieldName("x");       // 2. 设置过滤域名
    pass.setFilterLimits(robot_x_min, robot_x_max);     // 3. 设置过滤范围
    pass.filter(*filtered_assembled_pointCloud);
    // y
    pass.setInputCloud(filtered_assembled_pointCloud);          // 1. 设置输入源
    pass.setFilterFieldName("y");       // 2. 设置过滤域名
    pass.setFilterLimits(robot_y_min, robot_y_max);     // 3. 设置过滤范围
    pass.filter(*filtered_assembled_pointCloud);
    // z
    pass.setInputCloud(filtered_assembled_pointCloud);          // 1. 设置输入源
    pass.setFilterFieldName("z");       // 2. 设置过滤域名
    pass.setFilterLimits(robot_z_min, robot_z_max);     // 3. 设置过滤范围
    pass.filter(*filtered_assembled_pointCloud);

    clock_t time2 = clock();
//    filtered_assembled_pointCloud = current_assembled_map;

//    sensor_msgs::PointCloud2 msg;
//    pcl::toROSMsg(*filtered_assembled_pointCloud, msg);
//    msg.header.stamp = pose->header.stamp;
//    msg.header.frame_id = "map";
//    elevation_pc_pub.publish(msg);

    // 2. change point cloud to elevation map
//    grid_map::GridMap map({"elevation"});
//    map.setFrameId(grid_frame_id);
//    map.setPosition(grid_map::Position(grid_map_position_x,grid_map_position_y));
//    map.setGeometry(grid_map::Length(grid_map_size_x, grid_map_size_y),
//                    grid_map_resolution,
//                    grid_map::Position(grid_map_position_x, grid_map_position_y));
    grid_map::Position position_new(pose_stamped.pose.pose.position.x,
                                    pose_stamped.pose.pose.position.y);
    map.move(position_new);
    ROS_INFO_THROTTLE(
            2.0,
            "The center of the moved map is located at (%f, %f) in the %s frame.",
            map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

    for (unsigned int i = 0; i < filtered_assembled_pointCloud->size(); ++i) {
        auto &point = filtered_assembled_pointCloud->points[i];
        grid_map::Index index;
        grid_map::Position position(point.x, point.y);

        if (!map.getIndex(position, index))
            continue;  // Skip this point if it does not lie within the elevation map.

        auto &elevation = map.at("elevation", index);

        if (!map.isValid(index) || point_update) {
            // No prior information in elevation map or should update map in real
            // time, use measurement.
            elevation = point.z;
            continue;
        }
    }
    // Publish grid map.
    map.setTimestamp(pose_stamped.header.stamp.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    grid_pub.publish(message);


//    std::cout << "pubed" << std::endl;
    ROS_INFO_THROTTLE(10.0, "Grid map (timestamp %f) published.",
                      message.info.header.stamp.toSec());

    // publish elevation map
    // 1. get transform matrix
    tf::Vector3 pos(0.0, 0.0, 0.0);
    double roll, pitch, yaw;
    tf::Quaternion ori(pose_stamped.pose.pose.orientation.x, pose_stamped.pose.pose.orientation.y,
                       pose_stamped.pose.pose.orientation.z, pose_stamped.pose.pose.orientation.w);
    tf::Matrix3x3(ori).getRPY(roll, pitch, yaw);
    tf::Quaternion ori_yaw;
    ori_yaw.setRPY(0, 0, yaw);
    tf::Transform elevation_transformMatrix, pose_transformMatrix;
    elevation_transformMatrix.setRotation(ori_yaw);
//    std::cout << yaw << std::endl;
    pose_transformMatrix.setRotation(ori);

    // 2.get robot based elevation map and point cloud
    double ele{0.0f};
    double map_center[2]{map.getPosition().x(), map.getPosition().y()};

    detection::elevationSample ele_msg;
    ele_msg.header = pose->header;
    ele_msg.header.frame_id = "base";
    ele_msg.elevation.resize(width * length, 0.0);
    ele_msg.map_center.resize(2);
    ele_msg.map_center[0] = map_center[0];
    ele_msg.map_center[1] = map_center[1];

    // elevation map point cloud
    sensor_msgs::PointCloud2 elevation_map_pc_msgs;
    VPointCloud::Ptr pc_base_frame(new VPointCloud());

    // get elevation map in robot base frame.

    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            pos.setValue(X_grid[i][j], Y_grid[i][j], 0.0);
            pos = elevation_transformMatrix(pos);
            try {
                double pos_x{pos.x() + map_center[0]};
                double pos_y{pos.y() + map_center[1]};
                ele = map_inpainted.atPosition("elevation_inpainted",
                                               grid_map::Position(pos_x, pos_y));
                pcl::PointXYZ p(pos_x, pos_y, ele);
                pc_base_frame->push_back(p);
                ele_msg.elevation[i * width + j] = ele;
            } catch (std::out_of_range ex) {
//                ROS_WARN("out of range exception : %s", ex.what());
                continue;
            }
        }
    }
//    std::cout<<"pose: "<<pose->pose.position<<"#####"<<robot_position<<"#####"<<map_center[0]<<", "<<map_center[1]<<std::endl;
//    Eigen::VectorXd elevation_map{5041};
//    int center_idx = (int) ((5041 - 1) / 2);
//    double center_elevation = ele_msg.elevation[center_idx];
//    for (int i = 0; i < 5041; i++) {
//        elevation_map[i] = ele_msg.elevation[i] - center_elevation;
//    }
//    elevation_map = elevation_map.cwiseMax(-1).cwiseMin(1);
//    std::cout<<elevation_map.maxCoeff()<<"$$"<<elevation_map.minCoeff()<<std::endl;

    elevation_pub.publish(ele_msg);
    pcl::toROSMsg(*pc_base_frame, elevation_map_pc_msgs);
    elevation_map_pc_msgs.header.stamp = pose->header.stamp;
    elevation_map_pc_msgs.header.frame_id = "map";
    elevation_pc_pub.publish(elevation_map_pc_msgs);
//    std::cout<<"time2:"<<ros::Time::now()<<std::endl;
    clock_t time3 = clock();
//    std::cout<< "cloud time is: "<<static_cast<double>(time2-time1)/CLOCKS_PER_SEC*1000<<"ms"<<std::endl;
//    std::cout<< "ele time is: "<<static_cast<double>(time3-time2)/CLOCKS_PER_SEC*1000<<"ms"<<std::endl;

}

void UpdateGridMapInpainted(const grid_map_msgs::GridMap &grid_map_inpainted_msg) {
    grid_map::GridMapRosConverter::fromMessage(grid_map_inpainted_msg, map_inpainted);
}

void ReadParameter(ros::NodeHandle &nh_param) {
    nh_param.param<std::string>("point_sub_topic", point_sub_topic,
                                "/mapping_node/assembled_map");
    nh_param.param<std::string>("position_sub_topic", position_sub_topic,
                                "/mapping_node/scan2mapEstvelInterpolate_odometry");
    nh_param.param<std::string>("grid_pub_topic", grid_pub_topic, "/grid_map");
    nh_param.param<std::string>("elevation_pub_topic", elevation_pub_topic, "/elevation_map");
    nh_param.param<std::string>("elevation_pointCloud_pub_topic", elevation_pointCloud_pub_topic,
                                "/elevation_pointCloud_map");
    nh_param.param<std::string>("grid_frame_id", grid_frame_id, "/map");
    nh_param.param<double>("grid_map_size_x", grid_map_size_x, 5.0);
    nh_param.param<double>("grid_map_size_y", grid_map_size_y, 5.0);
    nh_param.param<double>("grid_map_position_x", grid_map_position_x, 0.0);
    nh_param.param<double>("grid_map_position_y", grid_map_position_y, 0.0);
    nh_param.param<double>("grid_map_resolution", grid_map_resolution, 0.05);
    nh_param.param<bool>("point_update", point_update, true);
    nh_param.param<bool>("position_update", position_update, false);
}

int main(int argc, char **argv) {
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_test");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_param("~");
    ReadParameter(nh_param);
    tf_listener_ptr = new tf::TransformListener(nh);

    // initialize robot based elevation map grid
    for (int i = 0; i < length; i++) {
        x[i] = (i - length / 2) * res;
    }
    for (int i = 0; i < width; i++) {
        y[i] = (i - width / 2) * res;
    }
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            X_grid[i][j] = x[i];
            Y_grid[i][j] = y[j];
        }
    }

    point_sub = nh.subscribe<sensor_msgs::PointCloud2>(point_sub_topic, 10, Point2grid);

    // filt point cloud to generate elevation map.
    position_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped >(position_sub_topic, 10, MapPositionUpdate);
    map_inpainted_sub = nh.subscribe("/grid_map_filter_demo/filtered_map", 10,
                                     UpdateGridMapInpainted);
    elevation_pub = nh.advertise<detection::elevationSample>(elevation_pub_topic, 10, true);
    elevation_pc_pub = nh.advertise<sensor_msgs::PointCloud2>(elevation_pointCloud_pub_topic, 10, true);
    grid_pub = nh.advertise<grid_map_msgs::GridMap>(grid_pub_topic, 10, true);
//
    // Initial grid map.
    map.setFrameId("map");
    map.setPosition(grid_map::Position(grid_map_position_x, grid_map_position_y));
    map.setGeometry(grid_map::Length(grid_map_size_x, grid_map_size_y),
                    grid_map_resolution,
                    grid_map::Position(grid_map_position_x, grid_map_position_y));
//        map.setGeometry(grid_map::Length(grid_map_size_x, grid_map_size_y),
//        grid_map_resolution);
    ROS_INFO(
            "Created map with  size %f x %f m (%i x %i cells). The center of the map "
            "is located at (%f, %f) in the %s frame.",
            map.getLength().x(), map.getLength().y(), map.getSize()(0),
            map.getSize()(1), map.getPosition().x(), map.getPosition().y(),
            map.getFrameId().c_str());

    // Work with grid map in a loop.
    ros::Rate rate(1000.0);

    while (nh.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
