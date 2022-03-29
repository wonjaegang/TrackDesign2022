#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *


def dynmixel_pose_pub():
    pub = rospy.Publisher('set_position', SetPosition, queue_size=10)
    rospy.init_node('dynmixel_pose_pub')
    rate = rospy.Rate(50)  # hz
    count = 0
    direction = 1
    msg = SetPosition()
    while not rospy.is_shutdown():
        goal_position = 2 * count
        msg.id = 1
        msg.position = goal_position
        pub.publish(msg)
        if count == 500:
            direction = -1
        elif count == 0:
            direction = 1
        else:
            pass
        count += direction
        rate.sleep()


def main():
    dynmixel_pose_pub()


if __name__ == '__main__':
    main()
