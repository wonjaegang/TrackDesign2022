#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub
import numpy as np
from scipy.spatial.transform import Rotation as R

# IKpy imports
from ikpy.utils import plot
from ikpy import chain

# msg
from std_msgs.msg import Float32MultiArray


def head_control(head_position):
    head_quat = R.from_quat(head_position[0:4])
    head_euler = head_quat.as_euler('xyz')
    #print(head_euler)
    return head_euler


def ik(target_l, target_r, initial_l, initial_r):
    inverse_l = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/new_meta_L_V22.xacro")
    inverse_l.active_links_mask[0] = False
    inverse_l.active_links_mask[7] = False

<<<<<<< HEAD
    inverse_r = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/new_meta_R_V22.xacro")
=======
    inverse_r = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/new_meta.xacro")
    #inverse_r = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/new_meta.xacro")
>>>>>>> f69943ea2a29e961fbe8f66ad0d2c119ee78bb35
    inverse_r.active_links_mask[0] = False
    inverse_r.active_links_mask[7] = False

    # 4X4 Transformation matrix
    def trans(goal_data):
        quat = R.from_quat(goal_data[0:4])
        rotation = quat.as_matrix()
        matrix_a = np.vstack((rotation, [0, 0, 0]))
        matrix_b = np.array([np.hstack((goal_data[4:7], 1))])
        transformation = np.hstack((matrix_a, matrix_b.T))
        return transformation

    # Calculate inverse kinematics
    ik_angle_l = inverse_l.inverse_kinematics_frame(trans(target_l), initial_position=initial_l)
    #print("Left angle : ", ik_angle_l[1:7])

    ik_angle_r = inverse_r.inverse_kinematics_frame(trans(target_r), initial_position=initial_r)
    #print("Right angle : ", ik_angle_r[1:7])

    
    # fig, ax = plot.init_3d_figure()
    # inverse_l.plot(ik_angle_l, ax, target_l[4:7])
    # inverse_r.plot(ik_angle_r, ax, target_r[4:7])
    # plot.show_figure()
    
    return ik_angle_l, ik_angle_r


<<<<<<< HEAD
def gripper(rjoy_A, rjoy_B, ljoy_X, ljoy_Y, gripper_right_pose, gripper_left_pose):
    rmax = np.pi / 6
    rmin = -np.pi / 2
    lmax = np.pi / 2
    lmin = -np.pi / 6
    
    gripper_speed = np.pi * 6 / 180

    if rjoy_B:
        gripper_right_pose += gripper_speed
        if gripper_right_pose > rmax:
            gripper_right_pose = rmax
    elif rjoy_A:
        gripper_right_pose -= gripper_speed
        if gripper_right_pose < rmin:
            gripper_right_pose = rmin

    if ljoy_Y:
        gripper_left_pose += gripper_speed
        if gripper_left_pose > lmax:
            gripper_left_pose = lmax
    elif ljoy_X:
        gripper_left_pose -= gripper_speed
        if gripper_left_pose < lmin:
            gripper_left_pose = lmin
    
    return gripper_right_pose, gripper_left_pose
        
=======
def gripper(joy_y):
    joy_min = -512
    joy_max = 512
    return (joy_y - joy_min) / (joy_max - joy_min) * np.pi / 2

>>>>>>> f69943ea2a29e961fbe8f66ad0d2c119ee78bb35

def main():
    rospy.init_node('inverse_kinematics')
    goal_position_pub = rospy.Publisher('/goal_position', Float32MultiArray, queue_size=1)
    goal = Float32MultiArray()
    oculus = oculus_sub()
    initial_l = [0, 0, 0, 0.1, 0.0, 0.0, 0.0, 0.0]
    initial_r = [0, 0, 0, 0.1, 0.0, 0.0, 0.0, 0.0]

    gripper_r = 0
    gripper_l = 0

    rospy.sleep(0.5)
    #print("Inverse Kinematics start!!!!!")

    while not rospy.is_shutdown():
<<<<<<< HEAD
        ik_goal_l, ik_goal_r = ik(oculus.left_pose, oculus.right_pose, initial_l, initial_r)
        head_goal = head_control(oculus.head_pose)
        gripper_r, gripper_l = gripper(oculus.rjoy_A, oculus.rjoy_B, oculus.ljoy_X, oculus.ljoy_Y, gripper_r, gripper_l)
        goal.data = np.concatenate([ik_goal_r[1:7],
                                    [gripper_r],
                                    ik_goal_l[1:7],
                                    [gripper_l],
=======
        #ik_goal_l, ik_goal_r = ik([0, 0, 0, 0.0001, -0.2, 0.9, 0.5],[0.108, -0.173, -0.705, 0.679, -0.2, -0.9, 0.5], initial_l,initial_r)
        #head_goal = head_control([0, -0.451, 0.544, 0.707, 0.0, -0.5, -0.2])
        ik_goal_l, ik_goal_r = ik(oculus.left_pose, oculus.right_pose, initial_l, initial_r)
        head_goal = head_control(oculus.head_pose)
        goal.data = np.concatenate([ik_goal_r[1:7],
                                    gripper(oculus.right_joy[1]),
                                    ik_goal_l[1:7],
                                    gripper(oculus.left_joy[1]),
>>>>>>> f69943ea2a29e961fbe8f66ad0d2c119ee78bb35
                                    head_goal[:0:-1]])
        initial_l = ik_goal_l
        initial_r = ik_goal_r
        goal_position_pub.publish(goal)


if __name__ == '__main__':
    main()
