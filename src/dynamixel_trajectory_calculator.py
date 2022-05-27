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
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectoryArray, queue_size=14)

        # Motor state values
        self.dxl_id_array = []
        self.goal_position = {}
        self.current_position = {}
        self.trajectory_array = {}

    def goal_position_callback(self, msg):
        def rad2deg(radian):
            return radian / math.pi * 180

        def index2id(index):
            return (index % 6) + (index // 6) * 10 + 1

        # Get DYNAMIXEL id
        self.dxl_id_array = [index2id(i) for i in range(len(msg.data))]

        # Save subscribed goal position
        self.goal_position = {dxl_id: rad2deg(data) for dxl_id, data in zip(self.dxl_id_array, msg.data)}

        # Get current position from server at DYNAMIXEL communicator
        self.current_position = {dxl_id: self.current_position_client(dxl_id).position for dxl_id in self.dxl_id_array}

        # Calculate Trajectory & Publish
        for dxl_id in self.dxl_id_array:

            # Calculate Trajectory
            trajectory = self.calculate_trajectory(dxl_id)

            # Struct message
            self.trajectory_array[dxl_id] = SetTrajectory()
            self.trajectory_array[dxl_id].id = dxl_id
            self.trajectory_array[dxl_id].position = trajectory['position']
            self.trajectory_array[dxl_id].velocity = trajectory['velocity']

        # Publish message
        trajectory_msg = SetTrajectoryArray()
        trajectory_msg.data = list(self.trajectory_array.values())
        self.trajectory_pub.publish(trajectory_msg)

        # Print data on terminal
        print("-" * 50)
        for dxl_id in self.dxl_id_array:
            print("DYNAMIXEL ID %d data state:" % dxl_id)
            print("    Goal Position(DGR):", round(self.goal_position[dxl_id], 4))
            print("    Current Position(DGR):", round(self.current_position[dxl_id], 4))
            print("    Published Position(DGR):", round(self.trajectory_array[dxl_id].position, 4))
            print("    Published Velocity(DPS):", round(self.trajectory_array[dxl_id].velocity, 4))
        print("-" * 50)

    def calculate_trajectory(self, dxl_id):
        # Should revise to P control
        initial_velocity_dict = {1:  60,   2: 60,  3: 400,  4: 200,  5: 400,  6: 400,
                                 11: 60,  12: 60, 13: 400, 14: 200, 15: 400, 16: 400,
                                 21: 400, 22: 400}

        position = self.goal_position[dxl_id]
        velocity = initial_velocity_dict[dxl_id]

        return {'position': position, 'velocity': velocity}


# -------------------------------------------- Main function -------------------------------------------------
def main():
    rospy.init_node('dynamixel_trajectory_calculator')
    trajectory_calculator = TrajectoryCalculator()
    rospy.spin()


if __name__ == '__main__':
    main()
