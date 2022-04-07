#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from geometry_msgs.msg import Point


class TrajectoryCalculator:
    def __init__(self):
        rospy.wait_for_service('/get_current_position')
        self.goal_position_sub = rospy.Subscriber('/pygame_pose', Point, self.goal_position_callback)
        self.current_position_client = rospy.ServiceProxy('/get_current_position', GetPosition)
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)

        # self.goal_position_check = False
        self.goal_position = None
        self.current_position = None
        self.trajectory_msg = SetTrajectory()

        self.followup_time = 0.5
        self.look_ahead_t = 0.1

    def goal_position_callback(self, msg):
        print("-" * 50)
        print("Subscribed Goal Position:", int(msg.x))
        # self.goal_position_check = True
        self.goal_position = int(msg.x)

        self.current_position = self.current_position_client(1)
        print("Requested Current Position:", self.current_position)
        #     ------    Should check if the code is running properly before this point.
        # Maybe... we need delay here.

        trajectory = self.calculate_trajectory()
        print("Calculated Trajectory:", trajectory)

        self.trajectory_msg.id = 1
        self.trajectory_msg.position = trajectory['position']
        self.trajectory_msg.position = trajectory['velocity']
        self.trajectory_pub.publish(self.trajectory_msg)
        print("Published Trajectory:", trajectory)
        print("-" * 50)
        # Maybe... we need delay here also.

    def calculate_trajectory(self):
        position = self.goal_position
        velocity = 0
        return {'position': position, 'velocity': velocity}


# ------------------------------- Function to create polynomial trajectory ---------------------------------
def trajectory_cubic(data_array, t):
    p1 = data_array[0]
    p2 = data_array[1]
    v1 = data_array[2]
    v2 = data_array[3]
    tg = data_array[4]

    a0 = p1
    a1 = v1
    a2 = 3 / (tg ** 2) * (p2 - p1) - 2 / tg * v1 - 1 / tg * v2
    a3 = -2 / (tg ** 3) * (p2 - p1) + 1 / (tg ** 2) * (v2 + v1)
    trajectory_position = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
    return trajectory_position


# -------------------------------------------- Main function -------------------------------------------------
def main():
    rospy.init_node('dynamixel_trajectory_calculator')
    trajectory_calculator = TrajectoryCalculator()
    rospy.spin()


if __name__ == '__main__':
    main()
