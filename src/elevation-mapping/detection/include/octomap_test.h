//
// Created by zdy on 23-12-12.
//

#ifndef SRC_OCTOMAP_TEST_H
#define SRC_OCTOMAP_TEST_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <assert.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


namespace octomap {
    bool fromOctomap(octomap::OcTree &octomap,
                     const std::string &layer,
                     grid_map::GridMap &gridMap,
                     const grid_map::Position3 *minPoint,
                     const grid_map::Position3 *maxPoint);

    class OCtreeProcessor {
    public:
        explicit OCtreeProcessor(ros::NodeHandle nh);

        void update();

        OcTree getSubOctomap();


    private:

        void parseParameter();

        void loadOCtreeFile();

        bool getQueryOccupy(point3d query);

        void sliceSubOctomap();

        bool convertOctomapToGridmap();

        void robotPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);

        ros::NodeHandle nh_;
        float subOctomap_range_x_ = 2.5f;
        float subOctomap_range_y_ = 2.5f;
        float subOctomap_range_z_ = 1.5f;
        int counter_ = 0;
        float octomap_resolution_{0.03f};
        OcTree octomap_{octomap_resolution_};
        OcTree octomap_ori_{octomap_resolution_};
        OcTree sub_octomap_{octomap_resolution_};
        std::vector<float> grid_x_, grid_y_, grid_z_;

        Eigen::Vector3f robot_position_;
        Eigen::Quaternionf robot_orientation_;
        grid_map::GridMap map_;
        std::string frame_id_;
        float map_length_, map_width_;
        float gridmap_resolution_;
        Eigen::VectorXf points_;  // elevation map for rviz.
        Eigen::VectorXf elevation_;

        std::string grid_pub_topic_;
        std::string point_cloud_pub_topic_;
        std::string robot_pos_sub_topic_;
        std::string octomap_filename_;
        geometry_msgs::PoseStamped pose_msg_;
        ros::Publisher gridMapPublisher_;
        ros::Publisher elevationPointsPublisher_;
        ros::Subscriber robotOdomSubscriber_;

        std::mutex msg_mtx_;
    };
} // namespace octomap

#endif //SRC_OCTOMAP_TEST_H
