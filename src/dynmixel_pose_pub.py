#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *


def dynmixel_pose_pub():
    speed_and_position_pub = rospy.Publisher('/set_position_and_speed', SetPosition, queue_size=10)
    pose_pub = rospy.Publisher('/set_position', SetPosition, queue_size=10)
    rospy.init_node('dynmixel_pose_pub')

    speed_and_position_msg = SetPosition()
    speed_and_position_msg.id = 1
    speed_and_position_msg.position = 500
    speed_and_position_pub.publish(speed_and_position_msg)
    # pose_msg = SetPosition()
    # pose_msg.id = 1
    # pose_msg.position = 1000
    # pose_pub.publish(pose_msg)


def main():
    dynmixel_pose_pub()


if __name__ == '__main__':
    main()
