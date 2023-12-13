//
// Created by zdy on 23-12-12.
//

#include "octomap_test.h"

namespace octomap {
    OCtreeProcessor::OCtreeProcessor(ros::NodeHandle nh) : nh_(nh) {
        parseParameter();
        loadOCtreeFile();
        gridMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>(grid_pub_topic_, 10, true);
        robotOdomSubscriber_ = nh_.subscribe<geometry_msgs::PoseStamped>(robot_pos_topic_, 10,
                                                                         &OCtreeProcessor::robotPositionCallback, this);
        robot_position_.setZero();
//        map_.setBasicLayers({"elevation"});
//        octomap_.setResolution(resolution_);
//        sub_octomap_.setResolution(resolution_);
        subOctomap_grid_x_.clear();
        subOctomap_grid_y_.clear();
        subOctomap_grid_z_.clear();
        float x0 = -subOctomap_range_x_ / 2.f;
        int x_num = (int) (subOctomap_range_x_ / resolution_);
        for (int i = 0; i < x_num; i++) {
            subOctomap_grid_x_.push_back(x0 + i * resolution_);
        }
        float y0 = -subOctomap_range_y_ / 2.f;
        int y_num = (int) (subOctomap_range_y_ / resolution_);
        for (int i = 0; i < y_num; i++) {
            subOctomap_grid_y_.push_back(y0 + i * resolution_);
        }
        float z0 = -subOctomap_range_z_ / 2.f;
        int z_num = (int) (subOctomap_range_z_ / resolution_);
        for (int i = 0; i < z_num; i++) {
            subOctomap_grid_z_.push_back(z0 + i * resolution_);
        }
    }

    void OCtreeProcessor::parseParameter() {
        nh_.param<std::string>("robot_pos_topic", robot_pos_topic_, "/odom/robot_position");
        nh_.param<std::string>("grid_pub_topic", grid_pub_topic_, "/gridmap_from_octomap");
        nh_.param<std::string>("octomap_filename", octomap_filename_,
                               "/home/zdy/maps/azure_livox_hand_2023-11-07-20-10-48.bt");
        nh_.param<std::string>("gridmap_frame_id", frame_id_, "map");
    }

    void OCtreeProcessor::loadOCtreeFile() {
        octomap_.clear();
        octomap_.readBinary(octomap_filename_);
        std::cout << "full map node num:" << octomap_.calcNumNodes() << std::endl;
    }

    void OCtreeProcessor::update() {
//        Eigen::Vector3f robot_position{robot_pos_x_, 0, 0};
//        sliceSubOctomap(robot_position);  // get suboctomap point by point.
        msg_mtx_.lock();
//        robot_position_.x() = 5. + 2 * std::cos(counter_ * 0.1);
//        robot_position_.y() = 0. + 2 * std::sin(counter_ * 0.1);
//        robot_position_.z() = 0.4;
        grid_map::Position3 min_bound{robot_position_.x() - subOctomap_range_x_ / 2, robot_position_.y() - subOctomap_range_y_ / 2,
                                      robot_position_.z() - subOctomap_range_z_ / 2};
        grid_map::Position3 max_bound{robot_position_.x() + subOctomap_range_x_ / 2, robot_position_.y() + subOctomap_range_y_ / 2,
                                      robot_position_.z() + subOctomap_range_z_ / 2};
        msg_mtx_.unlock();
//        std::cout << min_bound.transpose() << "\n" << max_bound.transpose() << std::endl;
        bool res = octomap::fromOctomap(octomap_, "elevation", map_, &min_bound, &max_bound);
        if (!res) {
            ROS_ERROR("Failed to call convert Octomap.");
            return;
        }
        map_.setFrameId(frame_id_);
//         Publish as grid map.
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
        gridMapPublisher_.publish(gridMapMessage);
        counter_++;
        std::cout<<"update"<<std::endl;
    }

    void OCtreeProcessor::sliceSubOctomap(const Eigen::Vector3f &robot_position) {
        float x{robot_position[0]};
        float y{robot_position[1]};
        float z{robot_position[2]};
        for (auto dx: subOctomap_grid_x_) {
            for (auto dy: subOctomap_grid_y_) {
                for (auto dz: subOctomap_grid_z_) {
                    octomap::point3d endpoint((float) x + dx, (float) y + dy, (float) z + dz);
                    if (getQueryOccupy(endpoint)) {
                        sub_octomap_.updateNode(endpoint, true);//将此区域设为被占据
                    } else {
                        sub_octomap_.updateNode(endpoint, false);
                    }
                }
            }
        }
        std::cout << "sub map node num:" << sub_octomap_.calcNumNodes() << std::endl;
    }

    bool OCtreeProcessor::getQueryOccupy(octomap::point3d query) {
        //OcTreeNode 使用OcTreeDataNode类（一些基础操作：深拷贝、浅拷贝、拷贝成员函数、删除成员函数）实现
        //OcTreeNode 存储的为概率的对数值
        //内置方法
        //getOccupancy() 获取观测为占据的概率
        //getLogOdds() 获取观测为占据概率的对数
        //getMeanChildLogOdds()
        //getMaxChildLogOdds()
        //addValue(p)    占据概率的对数加p
        OcTreeNode *Occupied_state = octomap_.search(query);
        if (Occupied_state != NULL) {
            if (Occupied_state->getOccupancy() > 0.5) return true;
        } else {
            return false;
        }
        return false;
    }

    OcTree OCtreeProcessor::getSubOctomap() {
        return sub_octomap_;
    }

    void OCtreeProcessor::robotPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
        std::lock_guard<std::mutex> lock(msg_mtx_);
        pose_msg_ = *msg;
        robot_position_[0] = msg->pose.position.x;
        robot_position_[1] = msg->pose.position.y;
        robot_position_[2] = msg->pose.position.z;
    }

    bool fromOctomap(octomap::OcTree &octomap,
                     const std::string &layer,
                     grid_map::GridMap &gridMap,
                     const grid_map::Position3 *minPoint,
                     const grid_map::Position3 *maxPoint) {

        // Copy octomap in order to expand any pruned occupied cells and maintain constness of input.
//        octomap::OcTree &octomapCopy(octomap);
        // Iterate through leaf nodes and project occupied cells to elevation map.
        // On the first pass, expand all occupied cells that are not at maximum depth.
        unsigned int max_depth = octomap.getTreeDepth();
        // Adapted from octomap octree2pointcloud.cpp.
        std::vector<octomap::OcTreeNode *> collapsed_occ_nodes;
        do {
            collapsed_occ_nodes.clear();
            for (octomap::OcTree::iterator it = octomap.begin(); it != octomap.end(); ++it) {
                if (octomap.isNodeOccupied(*it) && it.getDepth() < max_depth) {
                    collapsed_occ_nodes.push_back(&(*it));
                }
            }
            for (std::vector<octomap::OcTreeNode *>::iterator it = collapsed_occ_nodes.begin();
                 it != collapsed_occ_nodes.end(); ++it) {
#if OCTOMAP_VERSION_BEFORE_ROS_KINETIC
                (*it)->expandNode();
#else
                octomap.expandNode(*it);
#endif
            }
            // std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
        } while (collapsed_occ_nodes.size() > 0);

        // Set up grid map geometry.
        // TODO Figure out whether to center map.
        double resolution = octomap.getResolution();
        grid_map::Position3 minBound;
        grid_map::Position3 maxBound;
        octomap.getMetricMin(minBound(0), minBound(1), minBound(2));
        octomap.getMetricMax(maxBound(0), maxBound(1), maxBound(2));

        // User can provide coordinate limits to only convert a bounding box.
        octomap::point3d minBbx(minBound(0), minBound(1), minBound(2));
        if (minPoint) {
            minBbx = octomap::point3d((*minPoint)(0), (*minPoint)(1), (*minPoint)(2));
            minBound = grid_map::Position3(minBbx.x(), minBbx.y(), minBbx.z());
        }
        octomap::point3d maxBbx(maxBound(0), maxBound(1), maxBound(2));
        if (maxPoint) {
            maxBbx = octomap::point3d((*maxPoint)(0), (*maxPoint)(1), (*maxPoint)(2));
            maxBound = grid_map::Position3(maxBbx.x(), maxBbx.y(), maxBbx.z());
        }

        // create a larger grid map for octomap to avoid index out of range.
        grid_map::Length length = grid_map::Length(maxBound(0) - minBound(0) + 0.1, maxBound(1) - minBound(1) + 0.1);
        grid_map::Position position = grid_map::Position((maxBound(0) + minBound(0)) / 2.0,
                                                         (maxBound(1) + minBound(1)) / 2.0);
        gridMap.setGeometry(length, resolution, position);
//         std::cout << "grid map geometry: " << std::endl;
//         std::cout << "Length: [" << length(0) << ", " << length(1) << "]" << std::endl;
//         std::cout << "Position: [" << position(0) << ", " << position(1) << "]" << std::endl;
//         std::cout << "Resolution: " << resolution << std::endl;

        // Add elevation layer
        gridMap.add(layer);
        gridMap.setBasicLayers({layer});

        // For each voxel, if its elevation is higher than the existing value for the
        // corresponding grid map cell, overwrite it.
        // std::cout << "Iterating from " << min_bbx << " to " << max_bbx << std::endl;
        grid_map::Matrix &gridMapData = gridMap[layer];
        for (octomap::OcTree::leaf_bbx_iterator it = octomap.begin_leafs_bbx(minBbx, maxBbx),
                     end = octomap.end_leafs_bbx(); it != end; ++it) {
            if (octomap.isNodeOccupied(*it)) {
                octomap::point3d octoPos = it.getCoordinate();
                grid_map::Position position(octoPos.x(), octoPos.y());
                grid_map::Index index;
                gridMap.getIndex(position, index);
                // If no elevation has been set, use current elevation.
//                std::cout<<index<<std::endl;
                if (!gridMap.isValid(index)) {
                    gridMapData(index(0), index(1)) = octoPos.z();
                }
//                     Check existing elevation, keep higher.
                else {
                    if (gridMapData(index(0), index(1)) < octoPos.z()) {
                        gridMapData(index(0), index(1)) = octoPos.z();
                    }
                }
            }
        }

        return true;
    }

} // namespace octomap

int main(int argc, char **argv) {
    ros::init(argc, argv, "octree");
    ros::NodeHandle nh("~");
    octomap::OCtreeProcessor OCP(nh);
    ros::Rate rate(10);
    while (ros::ok()) {
        OCP.update();
        ros::spinOnce();
        rate.sleep();
    }
//    OCP.getSubOctomap().writeBinary("/home/zdy/maps/simple_tree.bt");
//    std::cout << "translation" << std::endl;
    return 0;
}

