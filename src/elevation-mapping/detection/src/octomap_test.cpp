//
// Created by zdy on 23-12-12.
//

#include "octomap_test.h"

namespace octomap {
    OCtreeProcessor::OCtreeProcessor(ros::NodeHandle nh) : nh_(nh) {
        parseParameter();
        loadOCtreeFile();
        gridMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>(grid_pub_topic_, 10);
        elevationPointsPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>(point_cloud_pub_topic_, 10, true);
        robotOdomSubscriber_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(robot_pos_sub_topic_, 10,
                                                                         &OCtreeProcessor::robotPositionCallback, this);
        robot_position_.setZero();
        robot_orientation_.setIdentity();
        grid_x_.clear();
        grid_y_.clear();
        grid_z_.clear();
        float x0 = -map_length_ / 2.f;
        int x_num = (int) (map_length_ / gridmap_resolution_);
        for (int i = 0; i < x_num; i++) {
            grid_x_.push_back(x0 + i * gridmap_resolution_);
        }
        float y0 = -map_width_ / 2.f;
        int y_num = (int) (map_width_ / gridmap_resolution_);
        for (int i = 0; i < y_num; i++) {
            grid_y_.push_back(y0 + i * gridmap_resolution_);
        }
        float z0 = subOctomap_range_z_ / 2.f;
        int z_num = (int) (subOctomap_range_z_ / octomap_resolution_);
        for (int i = 0; i < z_num; i++) {
            grid_z_.push_back(z0 - i * octomap_resolution_);
        }  // grid_z from up to down
    }

    void OCtreeProcessor::parseParameter() {
        nh_.param<std::string>("robot_pos_topic", robot_pos_sub_topic_, "/robot_base_pose_inter");
        nh_.param<std::string>("grid_pub_topic", grid_pub_topic_, "/gridmap_from_octomap");
        nh_.param<std::string>("pc_pub_topic", point_cloud_pub_topic_, "/elevation_from_octomap");
        nh_.param<std::string>("frame_id", frame_id_, "map");
        nh_.param<std::string>("octomap_filename", octomap_filename_,
                               "/home/zdy/msf_ws/maps/azure_livox_hand_2023-11-07.bt");
        nh_.param<float>("octomap_resolution", octomap_resolution_, 0.03);
        nh_.param<float>("gridmap_length", map_length_, 2);
        nh_.param<float>("gridmap_width", map_width_, 2);
        nh_.param<float>("gridmap_resolution", gridmap_resolution_, 0.05);

    }

    void OCtreeProcessor::loadOCtreeFile() {
        octomap_.clear();
        octomap_ori_.clear();
        octomap_.readBinary(octomap_filename_);
        octomap_ori_.readBinary(octomap_filename_);
        // Iterate through leaf nodes and project occupied cells to elevation map.
        // On the first pass, expand all occupied cells that are not at maximum depth.
        unsigned int max_depth = octomap_.getTreeDepth();
        // Adapted from octomap octree2pointcloud.cpp.
        std::vector<octomap::OcTreeNode *> collapsed_occ_nodes;
        do {
            collapsed_occ_nodes.clear();
            for (octomap::OcTree::iterator it = octomap_.begin(); it != octomap_.end(); ++it) {
                if (octomap_.isNodeOccupied(*it) && it.getDepth() < max_depth) {
                    collapsed_occ_nodes.push_back(&(*it));
                }
            }
            for (std::vector<octomap::OcTreeNode *>::iterator it = collapsed_occ_nodes.begin();
                 it != collapsed_occ_nodes.end(); ++it) {
#if OCTOMAP_VERSION_BEFORE_ROS_KINETIC
                (*it)->expandNode();
#else
                octomap_.expandNode(*it);
#endif
            }
            // std::cout << "Expanded " << collapsed_occ_nodes.size() << " nodes" << std::endl;
        } while (collapsed_occ_nodes.size() > 0);


        std::cout << "full map node num:" << octomap_.calcNumNodes() << std::endl;
    }

    void OCtreeProcessor::update() {
//        sliceSubOctomap();  // get suboctomap point by point.
        bool trans = convertOctomapToGridmap();
        if (!trans) {
            ROS_ERROR("Failed to convert Octomap to Gridmap.");
            std::cout << "error" << std::endl;
        }
        counter_++;
    }

    void OCtreeProcessor::sliceSubOctomap() {
        float x{robot_position_.x()};
        float y{robot_position_.y()};
        float z{robot_position_.z()};
        float qw{robot_orientation_.w()};
        float qx{robot_orientation_.x()};
        float qy{robot_orientation_.y()};
        float qz{robot_orientation_.z()};
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pub(new pcl::PointCloud<pcl::PointXYZ>);
        points_.setZero(map_length_ * map_width_ * 3);
        elevation_.setZero(map_length_ * map_width_);
        int idx = 0;
        float yaw = std::atan2(2 * (qw * qz + qx * qy), (1 - 2 * (qy * qy + qz * qz)));
        float cy = std::cos(yaw), sy = std::sin(yaw);
        for (auto dx: grid_x_) {
            for (auto dy: grid_y_) {
                for (auto dz: grid_z_) {
                    octomap::point3d endpoint((float) (x + cy * dx - sy * dy), (float) (y + sy * dx + cy * dy),
                                              (float) (z + dz));
                    if (getQueryOccupy(endpoint)) {
//                        sub_octomap_.updateNode(endpoint, true);//将此区域设为被占据
                        elevation_[idx] = endpoint.z();
                        points_[idx * 3] = endpoint.x();
                        points_[idx * 3 + 1] = endpoint.y();
                        points_[idx * 3 + 2] = endpoint.z();
                        continue;  // if searched a grid is occupied.
                    }
                }
                idx++;
            }
        }
        // pub elevation map point cloud.
        sensor_msgs::PointCloud2 elevation_map_pc_msgs;
        for (int i = 0; i < points_.size(); i += 3) {
            pcl::PointXYZ p(points_[i], points_[i + 1], points_[i + 2]);
            pc_pub->push_back(p);
        }
        pcl::toROSMsg(*pc_pub, elevation_map_pc_msgs);
        elevationPointsPublisher_.publish(elevation_map_pc_msgs);
    }

    bool OCtreeProcessor::convertOctomapToGridmap() {
        msg_mtx_.lock();
//        robot_position_.x() = 3.5324312448501587 + std::sin(counter_ * 0.1);
//        robot_position_.y() = 0. + 0.01 * std::sin(counter_ * 0.1);
//        robot_position_.z() = 0.4;
        grid_map::Position3 min_bound{robot_position_.x() - subOctomap_range_x_ / 2,
                                      robot_position_.y() - subOctomap_range_y_ / 2,
                                      robot_position_.z() - subOctomap_range_z_ / 2};
        grid_map::Position3 max_bound{robot_position_.x() + subOctomap_range_x_ / 2,
                                      robot_position_.y() + subOctomap_range_y_ / 2,
                                      robot_position_.z() + subOctomap_range_z_ / 2};
        std_msgs::Header msg_header = pose_msg_.header;
        msg_mtx_.unlock();
//        std::cout << min_bound.transpose() << "\n" << max_bound.transpose() << std::endl;
        bool res = octomap::fromOctomap(octomap_, "elevation", map_, &min_bound, &max_bound);
        if (!res) {
            ROS_ERROR("Failed to call convert Octomap.");
            return false;
        }
        map_.setFrameId(frame_id_);
//         Publish as grid map.
        grid_map_msgs::GridMap gridMapMessage;
        grid_map::GridMapRosConverter::toMessage(map_, gridMapMessage);
//        gridMapMessage.info.header = msg_header;
        gridMapMessage.info.header.stamp = msg_header.stamp;
        gridMapPublisher_.publish(gridMapMessage);
        return true;
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
        OcTreeNode *Occupied_state = octomap_ori_.search(query);
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

    void OCtreeProcessor::robotPositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
        std::lock_guard<std::mutex> lock(msg_mtx_);
        pose_msg_ = *msg;
        robot_position_.x() = msg->pose.pose.position.x;
        robot_position_.y() = msg->pose.pose.position.y;
        robot_position_.z() = msg->pose.pose.position.z;
        robot_orientation_.x() = msg->pose.pose.orientation.x;
        robot_orientation_.y() = msg->pose.pose.orientation.y;
        robot_orientation_.z() = msg->pose.pose.orientation.z;
        robot_orientation_.w() = msg->pose.pose.orientation.w;
    }

    bool fromOctomap(octomap::OcTree &octomap,
                     const std::string &layer,
                     grid_map::GridMap &gridMap,
                     const grid_map::Position3 *minPoint,
                     const grid_map::Position3 *maxPoint) {

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


void pc2octo(){
    std::string input_file = "/home/zdy/msf_ws/maps/1.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZ> ( input_file, cloud );

    std::cout<<"point cloud loaded, piont size = "<<cloud.points.size()<<std::endl;

    //声明octomap变量
    std::cout<<"copy data into octomap..."<<std::endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.03 );

    for (auto p:cloud.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary("/home/zdy/msf_ws/maps/1.bt");
    std::cout<<"done."<<std::endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "octree");
    ros::NodeHandle nh("~");
    octomap::OCtreeProcessor OCP(nh);
    ros::Rate rate(50);
    pc2octo();
    while (ros::ok()) {
        OCP.update();
        ros::spinOnce();
        rate.sleep();
    }
//    OCP.getSubOctomap().writeBinary("/home/zdy/maps/simple_tree.bt");
//    std::cout << "translation" << std::endl;
    return 0;
}

