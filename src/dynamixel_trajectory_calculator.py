#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from geometry_msgs.msg import Point


# class TrajectoryPub:
#     def __init__(self):
#         self.pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)
#         self.msg = SetTrajectory()
#         self.rate = 50
#
#     def pub_trajectory(self, dynamixel_id, goal_position, velocity):
#         self.msg.id = dynamixel_id
#         self.msg.position = goal_position
#         self.msg.velocity = velocity
#         self.pub.publish(self.msg)
#
#     def set_position(self, dynamixel_id, goal_position):
#         self.pub_trajectory(dynamixel_id, goal_position, 0)
#
#     # ----------------------------------- Methods to extract position & velocity -----------------------------------
#     # position: 0 ~ 1023
#     # velocity: -1023 ~ 1023  -> It will be quantized into 0 ~ 1023 in this function.
#     def cubic_velocity(self, dynamixel_id, current_position, set_position, current_velocity, set_velocity, tg):
#         def position2velocity(position):
#             return 50 * self.rate / 114 * position
#         goal_pos = set_position
#         p1 = current_position
#         p2 = goal_pos
#         v1 = current_velocity
#         v2 = set_velocity
#         data_array = [p1, p2, v1, v2, tg]
#
#         # Extract velocity from trajectory spline.
#         for i in range(int(tg * self.rate)):
#             if i == 0:
#                 velocity = v1
#             else:
#                 t = (i + 1) / self.rate
#                 velocity = int(position2velocity(trajectory_cubic(data_array, t) -
#                                                  trajectory_cubic(data_array, t - 1 / self.rate)) + 0.5)
#
#             # Change goal position when velocity direction is different with goal position.
#             velocity_magnitude = 1 if velocity == 0 else abs(velocity)
#             goal_pos = set_position
#             if (p2 - p1) * velocity < 0:
#                 if p2 > p1:
#                     goal_pos = 0
#                 else:
#                     goal_pos = 1023
#
#             self.pub_trajectory(dynamixel_id, goal_pos, velocity_magnitude)
#             rospy.Rate(self.rate).sleep()


class TrajectoryCalculator:
    def __init__(self):
        self.goal_position_sub = rospy.Subscriber('/pygame_pose', Point, self.goal_position_callback)
        self.current_position_sub = rospy.Subscriber('/current_position', SetPosition, self.current_position_callback)
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)
        self.goal_position_check = False
        self.goal_position = None
        self.current_position = None
        self.trajectory_msg = SetTrajectory()

    def goal_position_callback(self, msg):
        print("-" * 50)
        print("Subscribed Goal Position:", int(msg.x))
        self.goal_position_check = True
        self.goal_position = int(msg.x)

    def current_position_callback(self, msg):
        self.current_position = msg.position
        print("Subscribed Current Position:", self.current_position)
        if self.goal_position_check:
            trajectory = self.calculate_trajectory()
            self.trajectory_msg.id = 1
            self.trajectory_msg.position = trajectory['position']
            self.trajectory_msg.position = trajectory['velocity']
            self.trajectory_pub.publish(self.trajectory_msg)
            print("Published Trajectory:", trajectory)
        else:
            print("Error: goal_position was NOT subscribed yet.")
        self.goal_position_check = False
        print("-" * 50)

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