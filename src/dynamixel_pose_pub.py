#!/usr/bin/env python

import rospy
import numpy as np
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
import matplotlib.pyplot as plt


class TrajectoryPub:
    def __init__(self, node_name):
        self.node_name = node_name
        self.pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)
        self.msg = SetTrajectory()
        self.rate = 50

    def initialize_node(self):
        rospy.init_node(self.node_name)

    def pub_trajectory(self, dynamixel_id, goal_position, velocity):
        self.msg.id = dynamixel_id
        self.msg.position = goal_position
        self.msg.velocity = velocity
        self.pub.publish(self.msg)

    def set_position(self, dynamixel_id, goal_position):
        self.pub_trajectory(dynamixel_id, goal_position, 0)

    # ----------------------------------- Methods to extract position & velocity -----------------------------------
    def linear_position(self, dynamixel_id, current_position, set_position, tg):
        p1 = current_position
        p2 = set_position
        data_array = [p1, p2, tg]
        for i in range(int(tg * self.rate)):
            t = (i + 1) / self.rate
            position = int(trajectory_linear(data_array, t) + 0.5)
            self.pub_trajectory(dynamixel_id, position, 0)
            rospy.Rate(self.rate).sleep()

    def linear_velocity(self, dynamixel_id, current_position, set_position, tg):
        def position2velocity(position):
            return 50 * self.rate / 114 * position
        p1 = current_position
        p2 = set_position
        data_array = [p1, p2, tg]
        for i in range(int(tg * self.rate)):
            if i == 0:
                t = (i + 1) / self.rate
                velocity = abs(int(position2velocity(trajectory_linear(data_array, t + 1 / self.rate) -
                                                     trajectory_linear(data_array, t)) + 0.5))
            else:
                t = (i + 1) / self.rate
                velocity = abs(int(position2velocity(trajectory_linear(data_array, t) -
                                                     trajectory_linear(data_array, t - 1 / self.rate)) + 0.5))
            velocity_magnitude = 1 if velocity == 0 else abs(velocity)
            goal_pos = set_position
            if (p2 - p1) * velocity < 0:
                if p2 > p1:
                    goal_pos = 0
                else:
                    goal_pos = 1023
            self.pub_trajectory(dynamixel_id, goal_pos, velocity_magnitude)
            rospy.Rate(self.rate).sleep()

    def cubic_position(self, dynamixel_id, current_position, set_position, current_velocity, set_velocity, tg):
        goal_pos = set_position
        p1 = current_position
        p2 = goal_pos
        v1 = current_velocity
        v2 = set_velocity
        data_array = [p1, p2, v1, v2, tg]

        for i in range(int(tg * self.rate)):
            t = (i + 1) / self.rate
            position = int(trajectory_cubic(data_array, t) + 0.5)
            self.pub_trajectory(dynamixel_id, position, 0)
            rospy.Rate(self.rate).sleep()

    # position: 0 ~ 1023
    # velocity: -1023 ~ 1023  -> It will be quantized into 0 ~ 1023 in this function.
    def cubic_velocity(self, dynamixel_id, current_position, set_position, current_velocity, set_velocity, tg):
        def position2velocity(position):
            return 50 * self.rate / 114 * position
        goal_pos = set_position
        p1 = current_position
        p2 = goal_pos
        v1 = current_velocity
        v2 = set_velocity
        data_array = [p1, p2, v1, v2, tg]

        # Extract velocity from trajectory spline.
        for i in range(int(tg * self.rate)):
            if i == 0:
                velocity = v1
            else:
                t = (i + 1) / self.rate
                velocity = int(position2velocity(trajectory_cubic(data_array, t) -
                                                 trajectory_cubic(data_array, t - 1 / self.rate)) + 0.5)

            # Change goal position when velocity direction is different with goal position.
            velocity_magnitude = 1 if velocity == 0 else abs(velocity)
            goal_pos = set_position
            if (p2 - p1) * velocity < 0:
                if p2 > p1:
                    goal_pos = 0
                else:
                    goal_pos = 1023

            self.pub_trajectory(dynamixel_id, goal_pos, velocity_magnitude)
            rospy.Rate(self.rate).sleep()


# ------------------------------- Functions to create polynomial trajectory ---------------------------------
def trajectory_linear(data_array, t):
    p1 = data_array[0]
    p2 = data_array[1]
    tg = data_array[2]

    a0 = p1
    a1 = (p2 - p1) / tg
    trajectory_position = a0 + a1 * t
    return trajectory_position


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


def plot_trajectory(data_array, func):
    plot_point = 100
    dt = data_array[-1] / 100
    t_array = [dt * i for i in range(plot_point)]
    traj = [func(data_array, dt * i) for i in range(plot_point)]

    plt.plot(t_array, traj)
    plt.show()


# ----------------------------------- Main function -----------------------------------
def main():
    trajectory_pub = TrajectoryPub('dynamixel_trajectory_pub')
    trajectory_pub.initialize_node()

    dynamixel_id = 1
    start_pos = 0
    end_pos = 1023
    start_vel = 0
    end_vel = 0
    time_goal = 1
    while True:
        input_str = input()
        if input_str[:4] == 'rate':
            trajectory_pub.rate = int(input_str[4:])
            print("Rate changed to %dHz" % trajectory_pub.rate)
        elif input_str == 'lp':
            print("linear trajectory, position control")
            trajectory_pub.linear_position(dynamixel_id, start_pos, end_pos, time_goal)
        elif input_str == 'lv':
            print("linear trajectory, velocity control")
            trajectory_pub.linear_velocity(dynamixel_id, start_pos, end_pos, time_goal)
        elif input_str == 'cp':
            print("cubic trajectory, position control")
            trajectory_pub.cubic_position(dynamixel_id, start_pos, end_pos, start_vel, end_vel, time_goal)
        elif input_str == 'cv':
            print("cubic trajectory, velocity control")
            trajectory_pub.cubic_velocity(dynamixel_id, start_pos, end_pos, start_vel, end_vel, time_goal)
        elif input_str == 'i':
            print("Initialize position")
            trajectory_pub.set_position(dynamixel_id, start_pos)
        else:
            break
        rospy.Rate(0.5).sleep()


if __name__ == '__main__':
    main()
