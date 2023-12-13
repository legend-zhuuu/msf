//
// Created by zdy on 23-12-12.
//

#ifndef SRC_OCTOMAP_TEST_H
#define SRC_OCTOMAP_TEST_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>


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

        void sliceSubOctomap(const Eigen::Vector3f &robot_position);

        void robotPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);

        ros::NodeHandle nh_;
        float subOctomap_range_x_ = 2.4f;
        float subOctomap_range_y_ = 2.4f;
        float subOctomap_range_z_ = 1.2f;
        int counter_ = 0;
        float resolution_{0.03f};
        OcTree octomap_{resolution_};
        OcTree sub_octomap_{resolution_};
        std::vector<float> subOctomap_grid_x_, subOctomap_grid_y_, subOctomap_grid_z_;

        std::string robot_pos_topic_;
        std::string grid_pub_topic_;
        Eigen::Vector3f robot_position_;
        grid_map::GridMap map_;
        std::string frame_id_;
        std::string octomap_filename_;
        geometry_msgs::PoseStamped pose_msg_;
        ros::Publisher gridMapPublisher_;
        ros::Subscriber robotOdomSubscriber_;

        std::mutex msg_mtx_;
    };
} // namespace octomap

#endif //SRC_OCTOMAP_TEST_H
