#!/usr/bin/env python

import rospy
import math
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from std_msgs.msg import Float32MultiArray


# Angle: -180 ~ 180. Upper is 0.
class TrajectoryCalculator:
    def __init__(self):
        # Declare subscriber, client, and publisher
        rospy.wait_for_service('/get_current_position')
        self.goal_position_sub = rospy.Subscriber('/goal_position', Float32MultiArray, self.goal_position_callback)
        self.current_position_client = rospy.ServiceProxy('/get_current_position', GetPosition)
        self.trajectory_pub = rospy.Publisher('/set_trajectory', SetTrajectoryArray, queue_size=1)

        # Motor state values
        self.dxl_id_array = []
        self.goal_position = {}
        self.current_position = {}
        self.trajectory_array = {}

    def goal_position_callback(self, msg):
        def rad2deg(radian):
            return radian / math.pi * 180

        def index2id(index):
            return (index % 7) + (index // 7) * 10 + 1

        # Get DYNAMIXEL id
        # self.dxl_id_array = [index2id(i) for i in range(len(msg.data))]
<<<<<<< HEAD
        self.dxl_id_array = [index2id(i) for i in range(len(msg.data))]
=======
        self.dxl_id_array = [index2id(i) for i in range(len(msg.data))][:7]  # One Arm + Gripper. Should be changed
>>>>>>> f69943ea2a29e961fbe8f66ad0d2c119ee78bb35

        # Save subscribed goal position
        self.goal_position = {dxl_id: rad2deg(data) for dxl_id, data in zip(self.dxl_id_array, msg.data)}

        # # Get current position from server at DYNAMIXEL communicator
        # self.current_position= {dxl_id: self.current_position_client(dxl_id).position for dxl_id in self.dxl_id_array}

        # print(self.goal_position[1], self.goal_position[2], self.goal_position[11], self.goal_position[12])

        # # Temp code
        # if self.goal_position[1] > 0:
        #     if self.goal_position[2] > self.goal_position[1]:
        #         self.goal_position[2] = self.goal_position[1]
        # else:
        #     if self.goal_position[2] > 0:
        #         self.goal_position[2] = 0

        # if self.goal_position[11] < -30:
        #     if self.goal_position[12] < self.goal_position[11]:
        #         self.goal_position[12] = self.goal_position[11]
        # else:
        #     if self.goal_position[12] < 0:
        #         self.goal_position[2] = 0
            
        # Calculate Trajectory & Publish
        for dxl_id in self.dxl_id_array:

            # # Calculate Trajectory
            # trajectory = self.calculate_trajectory(dxl_id)
            # -> Delete Temporally for faster calculation

            # Struct message
            self.trajectory_array[dxl_id] = SetTrajectory()
            self.trajectory_array[dxl_id].id = dxl_id
            # self.trajectory_array[dxl_id].position = trajectory['position']
            self.trajectory_array[dxl_id].position = self.goal_position[dxl_id]
            # self.trajectory_array[dxl_id].velocity = trajectory['velocity']
            self.trajectory_array[dxl_id].velocity = 80

        # Publish message
        trajectory_msg = SetTrajectoryArray()
        trajectory_msg.data = list(self.trajectory_array.values())
        self.trajectory_pub.publish(trajectory_msg)

        # # Print data on terminal
        # print("-" * 50)
        # for dxl_id in self.dxl_id_array:
        #     print("DYNAMIXEL ID %d data state:" % dxl_id)
        #     print("    Goal Position(DGR):", round(self.goal_position[dxl_id], 4))
        #     print("    Current Position(DGR):", round(self.current_position[dxl_id], 4))
        #     print("    Published Position(DGR):", round(self.trajectory_array[dxl_id].position, 4))
        #     print("    Published Velocity(DPS):", round(self.trajectory_array[dxl_id].velocity, 4))

    def calculate_trajectory(self, dxl_id):
        # Should revise to P control
<<<<<<< HEAD
        initial_velocity_dict = {1: 100,   2: 100,  3: 100,  4: 100,  5: 100,  6: 100, 7: 0,
                                 11: 100, 12: 100, 13: 100, 14: 100, 15: 100, 16: 100, 17: 0,
                                 21: 100, 22: 100}
=======
        initial_velocity_dict = {1: 100,   2: 100,  3: 100,  4: 100,  5: 100,  6: 100, 7: 60}  # One Arm + Gripper. Should be changed
>>>>>>> f69943ea2a29e961fbe8f66ad0d2c119ee78bb35

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