#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose, Point
from std_msgs.msg import Header, Float64, ColorRGBA, Float64MultiArray

class Visualization():
    def __init__(self):

        self.left_hand_pub = rospy.Publisher('/rviz_left_hand', Marker, queue_size = 1)
        self.right_hand_pub = rospy.Publisher('/rviz_right_hand', Marker, queue_size = 1)
        self.head_pub = rospy.Publisher('/rviz_head', Marker, queue_size = 1)


    def head_marker(self,posestamped):

        rviz_msg_head = Marker(
            header = Header(frame_id = 'map', stamp = rospy.get_rostime()),
            ns = "head",
            id = 0,
            type = Marker.ARROW,
            lifetime = rospy.Duration(0.1),
            action = Marker.ADD,
            pose = posestamped.pose,
            scale = Vector3(x=2.0, y=0.3, z=0.2),
            color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        )

        self.head_pub.publish(rviz_msg_head)

    def left_hand_marker(self,posestamped):

        rviz_msg_left_hand = Marker(
            header = Header(frame_id = 'map', stamp = rospy.get_rostime()),
            ns = "left_hand",
            id = 1,
            type = Marker.ARROW,
            lifetime = rospy.Duration(0.1),
            action = Marker.ADD,
            pose = posestamped.pose,
            scale = Vector3(x=1.0, y=0.2, z=0.2),
            color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        )

        self.left_hand_pub.publish(rviz_msg_left_hand)

    def right_hand_marker(self,posestamped):

        rviz_msg_right_hand = Marker(
            header = Header(frame_id = 'map', stamp = rospy.get_rostime()),
            ns = "right_hand",
            id = 2,
            type = Marker.ARROW,
            lifetime = rospy.Duration(0.1),
            action = Marker.ADD,
            pose = posestamped.pose,
            scale = Vector3(x=1.0, y=0.2, z=0.2),
            color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        )

        self.right_hand_pub.publish(rviz_msg_right_hand)

def main():
    rospy.init_node('visualization', anonymous=True)
    oculus = oculus_sub()

    Vis = Visualization()

    rospy.sleep(0.5)
    print(" Start!! ")
    

    while not rospy.is_shutdown():
        #print(oculus.head_pose)
        Vis.left_hand_marker(oculus.left_pose)
        Vis.right_hand_marker(oculus.right_pose)
        Vis.head_marker(oculus.head_pose)
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
