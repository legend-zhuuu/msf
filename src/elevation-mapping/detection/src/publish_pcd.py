#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

time_stamp = 0


def ImuCallback(imu_msg):
    global time_stamp
    time_stamp = imu_msg.header.stamp


if __name__ == "__main__":

    rospy.init_node('cloudPointPub', anonymous=True)  # 定义节点
    if rospy.has_param('/pcd_file'):
        pcd_file = rospy.get_param('/pcd_file')
        print(f"load pcd from {pcd_file}")
    else:
        pcd_file = '/home/zdy/3.pcd'

    pub = rospy.Publisher('/global_map2', PointCloud2, queue_size=10)  # 定义话题
    pub_vel = rospy.Publisher('/estimate_velocity', Odometry, queue_size=10)  # 定义话题
    # imu_subscriber = rospy.Subscriber('/imu/data', Imu, ImuCallback)

    r = rospy.Rate(10)  # 10hz

    ##处理PCD点云
    # 得到文件夹下的所有文件名称
    pcd = o3d.io.read_point_cloud(pcd_file)
    # o3d.visualization.draw_geometries([pcd])
    exp_points = np.asarray(pcd.points)
    point_xyz = exp_points
    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # 构造消息
        msg = PointCloud2()
        if time_stamp:
            msg.header.stamp = time_stamp
        else:
            msg.header.stamp = rospy.Time().now()
        # msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = "map"
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * point_xyz.shape[0]
        msg.is_dense = False
        if len(point_xyz.shape) == 3:
            msg.height = point_xyz.shape[1]
            msg.width = point_xyz.shape[0]
        else:
            msg.height = 1
            msg.width = len(point_xyz)
        msg.data = np.asarray(point_xyz, np.float32).tobytes()
        pub.publish(msg)

        msg_vel = Odometry()
        msg_vel.header = msg.header
        msg_vel.pose.pose.position.x = 0
        msg_vel.pose.pose.position.y = 0
        msg_vel.pose.pose.position.z = 0
        msg_vel.pose.pose.orientation.x = 0
        msg_vel.pose.pose.orientation.y = 0
        msg_vel.pose.pose.orientation.z = 0
        msg_vel.pose.pose.orientation.w = 1
        msg_vel.twist.twist.linear.x = 0.2
        msg_vel.twist.twist.linear.y = 0
        msg_vel.twist.twist.linear.z = 0
        msg_vel.twist.twist.angular.x = 0
        msg_vel.twist.twist.angular.y = 0
        msg_vel.twist.twist.angular.z = 0
        pub_vel.publish(msg_vel)

        r.sleep()
