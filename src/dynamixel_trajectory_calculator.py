#!/usr/bin/env python

import rospy
import math
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from std_msgs.msg import Float32MultiArray


class TrajectoryCalculator:
    def __init__(self):
        # Declare subscriber, client, and publisher
        rospy.wait_for_service('/get_current_position')
        self.goal_position_sub = rospy.Subscriber('/goal_position', Float32MultiArray, self.goal_position_callback)
        self.current_position_client = rospy.ServiceProxy('/get_current_position', GetPosition)
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)

        # Motor state values
        self.goal_position = {}
        self.current_position = {}
        self.trajectory_msg = {i: SetTrajectory() for i in range(1, 7)}

    def goal_position_callback(self, msg):
        def rad2deg(radian):
            return radian / math.pi * 180

        # Save subscribed goal position
        self.goal_position = {i: rad2deg(msg.data[i - 1]) for i in range(1, 7)}

        # Get current position from server at DYNAMIXEL communicator
        self.current_position = {i: self.current_position_client(i).position for i in range(1, 7)}

        # Calculate Trajectory & Publish
        for i in range(1, 7):
            # Calculate Trajectory
            trajectory = self.calculate_trajectory(i)

            # Publish message
            self.trajectory_msg[i].id = i
            self.trajectory_msg[i].position = trajectory['position']
            self.trajectory_msg[i].velocity = trajectory['velocity']
            self.trajectory_pub.publish(self.trajectory_msg[i])

        # Print data on terminal
        print("-" * 50)
        for i in range(1, 7):
            print("DYNAMIXEL ID %d data state:" % i)
            print("    Goal Position(DGR):", round(self.goal_position[i], 4))
            print("    Current Position(DGR):", round(self.current_position[i], 4))
            print("    Published Position(DGR):", round(self.trajectory_msg[i].position, 4))
            print("    Published Velocity(DPS):", round(self.trajectory_msg[i].velocity, 4))
        print("-" * 50)

    def calculate_trajectory(self, dynamixel_id):
        velocity_dict = {1: 60, 2: 60, 3: 400, 4: 200, 5: 400, 6: 400}

        position = self.goal_position[dynamixel_id]
        velocity = velocity_dict[dynamixel_id]

        return {'position': position, 'velocity': velocity}


# -------------------------------------------- Main function -------------------------------------------------
def main():
    rospy.init_node('dynamixel_trajectory_calculator')
    trajectory_calculator = TrajectoryCalculator()
    rospy.spin()


if __name__ == '__main__':
    main()
