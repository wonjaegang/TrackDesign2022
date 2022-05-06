#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub
import numpy as np

# IKpy imports
from ikpy import chain
from ikpy.utils import plot

#오른손 기준
# + x: 아래
# + y: 앞
# + z: 바깥쪽



def main():
    oclus = oculus_sub()
    x = oclus.right_pose.pose.position.x

    a = chain.Chain.from_urdf_file("src/ik_study/urdf/meta_arm_2.xacro")
    print(a)
    frame_target = np.eye(4)
    joints = [0] * len(a.links)


    target = [0.3 , 0.3, -0.2]        
    frame_target[:3, 3] = target
    ik = a.inverse_kinematics(target, initial_position=joints)
    print("The angles of each joints are : ", a.inverse_kinematics(target))
    real_frame = a.forward_kinematics(a.inverse_kinematics(target),full_kinematics=True)
    print(real_frame)
    
    fig, ax = plot.init_3d_figure()
    a.plot(ik, ax, target=target)
    plot.show_figure()


if __name__ == '__main__':
    main()
