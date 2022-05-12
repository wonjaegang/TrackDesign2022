#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub
import numpy as np
from scipy.spatial.transform import Rotation as R

# IKpy imports
from ikpy import chain

#msg
from std_msgs.msg import Float32MultiArray



def ik(target):

    inverse = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/meta_arm.xacro")

    # 4X4 Transformation matrix
    quat = R.from_quat(target[0:4])
    rotation = quat.as_matrix()
    matrix_a = np.vstack((rotation,[0,0,0]))
    matrix_b = np.array([np.hstack((target[4:7],1))])
    transformation = np.hstack((matrix_a,matrix_b.T))

    # Calculate inverse kinematics
    ik_angle = inverse.inverse_kinematics_frame(transformation)
    goal_position = ik_angle[1:7]

    print("The angles of each joints are : ", goal_position)

    return goal_position



def main():
    rospy.init_node('inverse_kinematics')
    goal_position_pub = rospy.Publisher('/goal_position', Float32MultiArray,queue_size=10)
    goal = Float32MultiArray()
    oculus = oculus_sub()
    rospy.sleep(0.5)
    print("start!!!!!")

    while not rospy.is_shutdown():

        #goal.data = s.ik([-0.363, 0.609, 0.383, 0.592, 0.3, -0.1, -0.6])
        print("current pose", oculus.right_pose)
        goal.data = ik(oculus.right_pose)

        goal_position_pub.publish(goal)
        #rospy.sleep(0.1)

if __name__ == '__main__':
    main()
