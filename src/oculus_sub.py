#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import PoseStamped


class oculus_sub:

    def __init__(self):
        
        # subscriber (LiDAR, Odometry)
        self.head_pose_sub = rospy.Subscriber('/oculus/head_set/pose',PoseStamped,self.head_pose_callback,queue_size=1)
        self.left_pose_sub = rospy.Subscriber('/oculus/left_hand', PoseStamped,self.left_pose_callback,queue_size=1)
        self.right_pose_sub = rospy.Subscriber('/oculus/right_hand', PoseStamped,self.right_pose_callback,queue_size=1) 

    # callback
    def head_pose_callback(self, data):
        #msg = PoseStamped()
        data.pose.position.x = data.pose.position.x*10
        data.pose.position.y = data.pose.position.y*10
        data.pose.position.z = data.pose.position.z*10
        self.head_pose = data

    def left_pose_callback(self, data): 
        data.pose.position.x = data.pose.position.x*10
        data.pose.position.y = data.pose.position.y*10
        data.pose.position.z = data.pose.position.z*10
        self.left_pose = data

    def right_pose_callback(self, data): 
        data.pose.position.x = data.pose.position.x*10
        data.pose.position.y = data.pose.position.y*10
        data.pose.position.z = data.pose.position.z*10
        self.right_pose = data
