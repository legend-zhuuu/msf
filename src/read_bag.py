# coding:utf-8
# !/usr/bin/python

# Extract images from a bag file.
import os
import sys
import roslib
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np

class BagReader():
    def __init__(self):
        self.robot_base_pose_inter_dict = {"time_stamp": [], "x": [], "y": [], "z": []}
        self.lidar_pose_dict = {"time_stamp": [], "x": [], "y": [], "z": []}
        self.elevation_map_center_dict = {"time_stamp": [], "x": [], "y": []}
        self.plot_time =  40
        self.robot_pose_len = 50 * self.plot_time
        self.lidar_pose_len = 10 * self.plot_time
        self.elevation_len = 50 * self.plot_time

        with rosbag.Bag('/home/zdy/bags/test.bag', 'r') as bag:  # 要读取的bag文件；
            for topic, msg, t in bag.read_messages():
                if topic in ["/robot_base_pose_inter", "/lidar_pose"]:
                    timestr_ve = "%.4f" % msg.header.stamp.to_sec()
                    px = msg.pose.position.x
                    py = msg.pose.position.y
                    pz = msg.pose.position.z
                    # print(f"index_{id}, topic:{topic}, timestamp: {timestr_ve}, position: ({px}, {py}, {pz})")
                    if topic == "/robot_base_pose_inter":
                        self.robot_base_pose_inter_dict["time_stamp"].append(float(timestr_ve))
                        self.robot_base_pose_inter_dict["x"].append(px)
                        self.robot_base_pose_inter_dict["y"].append(py)
                        self.robot_base_pose_inter_dict["z"].append(pz)
                    else:
                        self.lidar_pose_dict["time_stamp"].append(float(timestr_ve))
                        self.lidar_pose_dict["x"].append(px)
                        self.lidar_pose_dict["y"].append(py)
                        self.lidar_pose_dict["z"].append(pz)
                elif topic == "/elevation_map":
                    timestr_ve = "%.4f" % msg.header.stamp.to_sec()
                    px = msg.map_center[0]
                    py = msg.map_center[1]
                    self.elevation_map_center_dict["time_stamp"].append(float(timestr_ve))
                    self.elevation_map_center_dict["x"].append(px)
                    self.elevation_map_center_dict["y"].append(py)
    def plot_pose(self):
        plt.figure(figsize=(20,15), dpi=80)
        plt.subplot(3, 1, 1)
        # plt.xticks(np.linspace(1698290640, 1698290662, 10,endpoint=True))
        plt.scatter(self.lidar_pose_dict["time_stamp"][:self.lidar_pose_len], self.lidar_pose_dict["x"][:self.lidar_pose_len], color="blue", linestyle="-")
        plt.scatter(self.robot_base_pose_inter_dict["time_stamp"][:self.robot_pose_len], self.robot_base_pose_inter_dict["x"][:self.robot_pose_len], color="red", linestyle="-")
        plt.scatter(self.elevation_map_center_dict["time_stamp"][:self.elevation_len], self.elevation_map_center_dict["x"][:self.elevation_len], color="black", linestyle="-")
        plt.subplot(3, 1, 2)
        # # plt.xticks(np.linspace(1698290640, 1698290662, 10,endpoint=True))
        plt.scatter(self.lidar_pose_dict["time_stamp"][:self.lidar_pose_len], self.lidar_pose_dict["y"][:self.lidar_pose_len], color="blue", linestyle="-")
        plt.scatter(self.robot_base_pose_inter_dict["time_stamp"][:self.robot_pose_len], self.robot_base_pose_inter_dict["y"][:self.robot_pose_len], color="red", linestyle="-")
        plt.scatter(self.elevation_map_center_dict["time_stamp"][:self.elevation_len], self.elevation_map_center_dict["y"][:self.elevation_len], color="black", linestyle="-")
        #
        plt.subplot(3, 1, 3)
        # plt.xticks(np.linspace(1698290640, 1698290662, 10,endpoint=True))
        plt.scatter(self.lidar_pose_dict["time_stamp"][:self.lidar_pose_len], self.lidar_pose_dict["z"][:self.lidar_pose_len], color="blue", linestyle="-")
        plt.scatter(self.robot_base_pose_inter_dict["time_stamp"][:self.robot_pose_len], self.robot_base_pose_inter_dict["z"][:self.robot_pose_len], color="red", linestyle="-")

        plt.show()


if __name__ == '__main__':
    try:
        bag_reader = BagReader()
        bag_reader.plot_pose()
    except rospy.ROSInterruptException:
        pass
