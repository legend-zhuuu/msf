#!/usr/bin/env python3

import argparse
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('topic', type=str)
    parser.add_argument('velocity', type=str)
    args = parser.parse_args()
    vel = eval(args.velocity)

    rospy.init_node('cmd_vel_publisher')
    rate = rospy.Rate(50)
    pub = rospy.Publisher(args.topic, Twist, queue_size=1)
    print(f'Start publishing: vx={vel[0]}, vy={vel[1]}, vz={vel[2]}')
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = vel[0]
        msg.linear.y = vel[1]
        msg.angular.z = vel[2]
        pub.publish(msg)
        rate.sleep()
