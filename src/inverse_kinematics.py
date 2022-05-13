#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub
import numpy as np
from scipy.spatial.transform import Rotation as R

# IKpy imports
from ikpy import chain, inverse_kinematics
from ikpy.utils import plot

#msg
from std_msgs.msg import Float32MultiArray

def ik(target, initial):

    inverse = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/meta_arm.xacro")
    print(inverse)
    inverse.active_links_mask[0] = False
    inverse.active_links_mask[7] = False

    # 4X4 Transformation matrix
    def trans(goal_data):
        quat = R.from_quat(goal_data[0:4])
        rotation = quat.as_matrix()
        matrix_a = np.vstack((rotation,[0,0,0]))
        matrix_b = np.array([np.hstack((goal_data[4:7],1))])
        transformation = np.hstack((matrix_a,matrix_b.T))
        return transformation

    # Calculate inverse kinematics
    ik_angle = inverse.inverse_kinematics_frame(trans(target),initial_position = initial)
    goal_position = ik_angle
    print("The angles of each joints are : ", goal_position[1:7])

    '''
    fig, ax = plot.init_3d_figure()
    inverse.plot(ik_angle, ax, target[4:7])
    plot.show_figure()
    '''
    return goal_position



def main():
    rospy.init_node('inverse_kinematics')
    goal_position_pub = rospy.Publisher('/goal_position', Float32MultiArray,queue_size=10)
    goal = Float32MultiArray()
    oculus = oculus_sub()
    initial = [0, 0, 0, 0.0001, -0.0001, 0.0001, 0.0001, 0]
    rospy.sleep(0.5)
    print("start!!!!!")

    while not rospy.is_shutdown():

        #ik_goal = ik([0, -0.451, 0.544, 0.707, 0.0, -0.5, -0.2], initial)
        #print("current pose", oculus.right_pose)
        ik_goal = ik(oculus.right_pose, initial)
        goal.data = ik_goal[1:7]
        initial = ik_goal

        goal_position_pub.publish(goal)

if __name__ == '__main__':
    main()
