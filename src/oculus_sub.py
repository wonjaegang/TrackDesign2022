#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import PoseStamped

class oculus_sub:

    def __init__(self):
        
        # subscriber (LiDAR, Odometry)
        self.head_pose_sub = rospy.Subscriber('/oculus/head_set/pose',PoseStamped,self.head_pose_callback,queue_size=1)
        self.left_pose_sub = rospy.Subscriber('/oculus/lpoint', PoseStamped,self.left_pose_callback,queue_size=1)
        self.right_pose_sub = rospy.Subscriber('/oculus/rpoint', PoseStamped,self.right_pose_callback,queue_size=1) 
        
    # callback
    def head_pose_callback(self, data):        
        pz = data.pose.position.x
        px = data.pose.position.y
        py = data.pose.position.z
        ox = data.pose.orientation.x
        oy = data.pose.orientation.y
        oz = data.pose.orientation.z
        ow = data.pose.orientation.w
        self.head_pose = [ox, oy, oz, ow, px, py, pz ]

    def left_pose_callback(self, data): 
        pz = data.pose.position.x - 0.3
        px = data.pose.position.y - 0.3
        py = data.pose.position.z - 1.1
        oz = data.pose.orientation.x
        ox = data.pose.orientation.y
        oy = data.pose.orientation.z
        ow = data.pose.orientation.w
        self.left_pose = [ox, oy, oz, ow, px, py, pz ]

    def right_pose_callback(self, data): 
        pz = data.pose.position.x - 0.3
        px = data.pose.position.y + 0.3
        py = data.pose.position.z - 1.1
        oz = data.pose.orientation.x
        ox = data.pose.orientation.y
        oy = data.pose.orientation.z
        ow = data.pose.orientation.w
        self.right_pose = [ox, oy, oz, ow, px, py, pz ]