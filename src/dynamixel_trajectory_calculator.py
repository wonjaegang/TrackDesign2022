#!/usr/bin/env python

import rospy
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class TrajectoryCalculator:
    def __init__(self):
        # Declare subscriber, client, and publisher
        rospy.wait_for_service('/get_current_position')
        self.goal_position_sub = rospy.Subscriber('/pygame_pose', Point, self.goal_position_callback)
        self.current_position_client = rospy.ServiceProxy('/get_current_position', GetPosition)
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)

        # Motor state values
        self.goal_position = {x: None for x in range(1, 7)}
        self.current_position = {x: None for x in range(1, 7)}
        self.trajectory_msg = SetTrajectory()

        # Control values
        self.followup_time = 0.5
        self.look_ahead_t = 0.05
        self.derivative_t = 0.01

    def goal_position_callback(self, msg):
        # Save subscribed goal position
        print("-" * 50)
        print("Subscribed Goal Position:", int(msg.x))
        # Should change the parameter later
        for i in range(1, 7):
            self.goal_position[i] = int(msg.x)

        # Get current position from server ar DYNAMIXEL communicator
        # Should change the parameter later
        for i in range(1, 3):
            self.current_position = self.current_position_client(i).position
            print("ID %d Requested Current Position:" % i, self.current_position)

        # Should change the parameter later
        for i in range(1, 3):
            # Calculate Trajectory
            trajectory = self.calculate_trajectory(i)

            # Publish message
            self.trajectory_msg.id = i
            self.trajectory_msg.position = trajectory['position']
            self.trajectory_msg.velocity = trajectory['velocity']
            self.trajectory_pub.publish(self.trajectory_msg)
            print("ID %d Published Trajectory:" % i, trajectory)
            print("-" * 50)

    def calculate_trajectory(self, dynamixel_id):
        def pose2vel(position_derivative):
            return 50 / self.followup_time / 114 * position_derivative

        # linear position control
        position = self.goal_position[dynamixel_id]
        velocity = 0

        # # linear velocity control
        # position = self.goal_position
        # velocity = abs(int(pose2vel((self.goal_position - self.current_position) / self.followup_time) + 0.5))
        # velocity = 1 if velocity == 0 else velocity

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
