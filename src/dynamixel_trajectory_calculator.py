#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from geometry_msgs.msg import Point


class TrajectoryCalculator:
    def __init__(self):
        # Declare subscriber, client, and publisher
        rospy.wait_for_service('/get_current_position')
        self.goal_position_sub = rospy.Subscriber('/pygame_pose', Point, self.goal_position_callback)
        self.current_position_client = rospy.ServiceProxy('/get_current_position', GetPosition)
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)

        # Motor state values
        self.goal_position = None
        self.current_position = None
        self.trajectory_msg = SetTrajectory()

        # Control values
        self.followup_time = 0.2
        self.look_ahead_t = 0.05
        self.derivative_t = 0.01

    def goal_position_callback(self, msg):
        # Save subscribed goal position
        print("-" * 50)
        print("Subscribed Goal Position:", int(msg.x))
        self.goal_position = int(msg.x)

        # Get current position from server ar DYNAMIXEL communicator
        self.current_position = self.current_position_client(1).position
        print("Requested Current Position:", self.current_position)

        # Calculate Trajectory
        trajectory = self.calculate_trajectory()

        # Publish message
        self.trajectory_msg.id = 1
        self.trajectory_msg.position = trajectory['position']
        self.trajectory_msg.velocity = trajectory['velocity']
        self.trajectory_pub.publish(self.trajectory_msg)
        print("Published Trajectory:", trajectory)
        print("-" * 50)

    def calculate_trajectory(self):
        def pose2vel(position_derivative):
            return 50 / self.derivative_t / 114 * position_derivative

        # linear position control
        position = self.goal_position
        velocity = 0

        return {'position': position, 'velocity': velocity}


# ------------------------------- Function to create polynomial trajectory ---------------------------------
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


# -------------------------------------------- Main function -------------------------------------------------
def main():
    rospy.init_node('dynamixel_trajectory_calculator')
    trajectory_calculator = TrajectoryCalculator()
    rospy.spin()


if __name__ == '__main__':
    main()
