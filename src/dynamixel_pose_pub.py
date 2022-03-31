#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
import matplotlib.pyplot as plt


class TrajectoryPub:
    def __init__(self, node_name):
        self.node_name = node_name
        self.pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)
        self.msg = SetTrajectory()
        self.rate = 10

    def initialize_node(self):
        rospy.init_node(self.node_name)

    def pub_trajectory(self, dynamixel_id, goal_position, velocity):
        self.msg.id = dynamixel_id
        self.msg.position = goal_position
        self.msg.velocity = velocity
        self.pub.publish(self.msg)


# ----------------------------------- Functions to plan Trajectory -----------------------------------
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


def trajectory_quintic(data_array, t):
    pass


def plot_trajectory(data_array, func):
    plot_point = 100
    dt = data_array[-1] / 100
    t_array = [dt * i for i in range(plot_point)]
    traj = [func(data_array, dt * i) for i in range(plot_point)]

    plt.plot(t_array, traj)
    plt.show()


# ----------------------------------- Functions to extract position & velocity -----------------------------------
# Functions that make polynomial spline for the trajectory of Dynamixel.
def linear_position(set_position, tg):
    trajectory_pub = TrajectoryPub('dynamixel_trajectory_pub')
    trajectory_pub.initialize_node()

    goal_pos = set_position
    p1 = 0
    p2 = goal_pos
    data_array = [p1, p2, tg]
    for i in range(int(tg * trajectory_pub.rate)):
        t = (i + 1) / trajectory_pub.rate
        position = int(trajectory_linear(data_array, t) + 0.5)
        trajectory_pub.pub_trajectory(1, position, 0)
        rospy.Rate(trajectory_pub.rate).sleep()


def linear_velocity(set_position, tg):
    def position2velocity(position):
        return 50 * trajectory_pub.rate / 114 * position
    trajectory_pub = TrajectoryPub('dynamixel_trajectory_pub')
    trajectory_pub.initialize_node()

    goal_pos = set_position
    p1 = 0
    p2 = goal_pos
    data_array = [p1, p2, tg]
    for i in range(int(tg * trajectory_pub.rate)):
        if i == 0:
            t = (i + 1) / trajectory_pub.rate
            velocity = abs(int(position2velocity(trajectory_linear(data_array, t + 1 / trajectory_pub.rate) -
                                                 trajectory_linear(data_array, t)) + 0.5))
        else:
            t = (i + 1) / trajectory_pub.rate
            velocity = abs(int(position2velocity(trajectory_linear(data_array, t) -
                                                 trajectory_linear(data_array, t - 1 / trajectory_pub.rate)) + 0.5))
        if velocity == 0:
            velocity = 1
        trajectory_pub.pub_trajectory(1, goal_pos, velocity)
        rospy.Rate(trajectory_pub.rate).sleep()


def cubic_position(set_position, tg):
    trajectory_pub = TrajectoryPub('dynamixel_trajectory_pub')
    trajectory_pub.initialize_node()

    goal_pos = set_position
    p1 = 0
    p2 = goal_pos
    v1 = 0
    v2 = 0
    data_array = [p1, p2, v1, v2, tg]

    for i in range(int(tg * trajectory_pub.rate)):
        t = (i + 1) / trajectory_pub.rate
        position = int(trajectory_cubic(data_array, t) + 0.5)
        trajectory_pub.pub_trajectory(1, position, 0)
        rospy.Rate(trajectory_pub.rate).sleep()


def cubic_velocity(set_position, tg):
    def position2velocity(position):
        return 50 * trajectory_pub.rate / 114 * position
    trajectory_pub = TrajectoryPub('dynamixel_trajectory_pub')
    trajectory_pub.initialize_node()

    goal_pos = set_position
    p1 = 0
    p2 = goal_pos
    v1 = 0
    v2 = 0
    data_array = [p1, p2, v1, v2, tg]
    for i in range(int(tg * trajectory_pub.rate)):
        if i == 0:
            velocity = v1
        else:
            t = (i + 1) / trajectory_pub.rate
            velocity = abs(int(position2velocity(trajectory_cubic(data_array, t) -
                                                 trajectory_cubic(data_array, t - 1 / trajectory_pub.rate)) + 0.5))
        if velocity == 0:
            velocity = 1
        trajectory_pub.pub_trajectory(1, goal_pos, velocity)
        rospy.Rate(trajectory_pub.rate).sleep()


def main():
    cubic_position(0, 0.5)
    rospy.Rate(0.5).sleep()

    linear_position(1023, 1)
    rospy.Rate(0.5).sleep()

    cubic_position(0, 0.5)
    rospy.Rate(0.5).sleep()

    linear_velocity(1023, 1)
    rospy.Rate(0.5).sleep()

    cubic_position(0, 0.5)
    rospy.Rate(0.5).sleep()

    cubic_position(1023, 1)
    rospy.Rate(0.5).sleep()

    cubic_position(0, 0.5)
    rospy.Rate(0.5).sleep()

    cubic_velocity(1023, 1)
    rospy.Rate(0.5).sleep()


if __name__ == '__main__':
    main()
