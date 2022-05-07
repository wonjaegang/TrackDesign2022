#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub
import numpy as np
from scipy.spatial.transform import Rotation as R


# IKpy imports
from ikpy import chain
from ikpy.utils import plot

#msg
from std_msgs.msg import Float32MultiArray


class set:
    def __init__(self):
        oculus = oculus_sub()

        self.goal_position_pub = rospy.Publisher('/goal_position', Float32MultiArray,queue_size=10)

        self.goal_position = []

    def ik(self,target):

        inverse = chain.Chain.from_urdf_file("src/TrackDesign2022/urdf/meta_arm.xacro")

        # 4X4 Transformation matrix
        rq = R.from_quat(target[0:4])
        rm = rq.as_matrix()
        ma = np.vstack((rm,[0,0,0]))
        mb = np.array([np.hstack((target[4:7],1))])
        tm = np.hstack((ma,mb.T))

        # Calculate inverse kinematics
        ik = inverse.inverse_kinematics_frame(tm)
        goal_position = ik[1:7]
        print("The angles of each joints are : ", goal_position)

        # plot
        fig, ax = plot.init_3d_figure()
        inverse.plot(ik, ax, target[4:7])
        plot.show_figure()

        return goal_position



def main():
    rospy.init_node('inverse_kinematics')
    goal = Float32MultiArray()
    s = set()
    aa = s.ik([-0.363, 0.609, 0.383, 0.592, 0.1, -0.1, -0.5])
    aa[3] = - aa[3]
    aa[2] = aa[2]-1.570796
    aa[4] = aa[4]-1.570796
    goal.data = aa
    
    s.goal_position_pub.publish(goal)

if __name__ == '__main__':
    main()
