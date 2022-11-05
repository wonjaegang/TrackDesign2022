#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

class oculus_sub:

    def __init__(self):
        
        # subscriber (LiDAR, Odometry)
        self.head_pose_sub = rospy.Subscriber('/oculus/head_set/pose',PoseStamped,self.head_pose_callback,queue_size=1)
        self.left_pose_sub = rospy.Subscriber('/oculus/lpoint', PoseStamped,self.left_pose_callback,queue_size=1)
        self.right_pose_sub = rospy.Subscriber('/oculus/rpoint', PoseStamped,self.right_pose_callback,queue_size=1)
        self.controller_sub = rospy.Subscriber('/oculus/controller', Joy,self.controller_callback,queue_size=1)  
        self.head_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.left_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.right_pose = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.rjoy_A = 0
        self.rjoy_B = 0
        self.ljoy_X = 0
        self.ljoy_Y = 0
        self.right_joy_sub = rospy.Subscriber('/oculus/rjoy', PoseStamped, self.right_joy_callback, queue_size=1)
        self.left_joy_sub = rospy.Subscriber('/oculus/ljoy', PoseStamped, self.left_joy_callback, queue_size=1)
        
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
        #self.head_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def controller_callback(self, data):
        self.rjoy_A = data.buttons[0]
        self.rjoy_B = data.buttons[1]
        self.ljoy_X = data.buttons[2]
        self.ljoy_Y = data.buttons[3]

    def left_pose_callback(self, data): 
        pz = data.pose.position.x - 0.25
        px = data.pose.position.y - 0.2
        py = data.pose.position.z - 1.3
        oz = data.pose.orientation.x
        ox = data.pose.orientation.y
        oy = data.pose.orientation.z
        ow = data.pose.orientation.w
        self.left_pose = [ox, oy, oz, ow, 1.2*px, 1.2*py, 1.2*pz ]

    def right_pose_callback(self, data): 
        pz = data.pose.position.x - 0.25
        px = data.pose.position.y + 0.2
        py = data.pose.position.z - 1.3
        self.left_pose = [ox, oy, oz, ow, px, py, pz ]
        #self.left_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def right_pose_callback(self, data):
        pz = data.pose.position.x - 0.3
        px = data.pose.position.y + 0.3
        py = data.pose.position.z - 1.1
        oz = data.pose.orientation.x
        ox = data.pose.orientation.y
        oy = data.pose.orientation.z
        ow = data.pose.orientation.w
        self.right_pose = [ox, oy, oz, ow, 1.2*px, 1.2*py, 1.2*pz ]
        self.right_pose = [ox, oy, oz, ow, px, py, pz ]
        #self.right_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Temp method. Should check datatype of OQ2's joystick.
    def right_joy_callback(self, data):
        self.right_joy = [data.x, data.y]

    def left_joy_callback(self, data):
        self.left_joy = [data.x, data.y]
