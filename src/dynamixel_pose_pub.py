#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *


def dynamixel_pose_pub():
    trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)
    position_pub = rospy.Publisher('/set_position', SetPosition, queue_size=10)
    rospy.init_node('dynamixel_pose_pub')

    trajectory_msg = SetTrajectory()
    trajectory_msg.id = 1
    trajectory_msg.position = 512
    trajectory_msg.velocity = 300
    trajectory_pub.publish(trajectory_msg)
    # position_msg = SetPosition()
    # position_msg.id = 1
    # position_msg.position = 1000
    # position_pub.publish(position_msg)


def main():
    dynamixel_pose_pub()


if __name__ == '__main__':
    main()
