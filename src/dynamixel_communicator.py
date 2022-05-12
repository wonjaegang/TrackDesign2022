#!/usr/bin/env python

import rospy
from dynamixel_sdk import *
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *

# Control table address
ADDRESS = {'AX18A': {'TORQUE_ENABLE': 24,
                     'GOAL_POSITION': 30,
                     'PRESENT_POSITION': 36},
           'XM430': {'TORQUE_ENABLE': 64,
                     'GOAL_POSITION': 116,
                     'PRESENT_POSITION': 132}}

# Protocol version
PROTOCOL_VERSION = 1.0             # protocol of AX-18A. Depends on motor.

# Default setting
BAUD_RATE = 1000000                 # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'        # Check which port is being used on your controller

# Actuator initial setting. Use ID as keys.
ACTUATOR_SETTING = {1: {'name': 'AX18A',
                        'initial_position_DGR': 0},
                    2: {'name': 'AX18A',
                        'initial_position_DGR': 0},
                    3: {'name': 'AX18A',
                        'initial_position_DGR': 0},
                    4: {'name': 'AX18A',
                        'initial_position_DGR': 90},
                    5: {'name': 'AX18A',
                        'initial_position_DGR': 0},
                    6: {'name': 'AX18A',
                        'initial_position_DGR': 0}}

portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


# ---------------------------------- Functions to initiate DYNAMIXEL ----------------------------------
def open_serial_port():
    try:
        portHandler.openPort()
        print("Succeeded to open the port, current port: %s" % DEVICE_NAME)
    except:
        print("Failed to open the port")
        quit()


def set_baudrate():
    try:
        portHandler.setBaudRate(BAUD_RATE)
        print("Succeeded to change the baudrate, current baudrate: %d" % BAUD_RATE)
    except:
        print("Failed to change the baudrate")
        quit()


def enable_torque(dynamixel_id):
    dxl_comm_result, dxl_error =\
        packetHandler.write1ByteTxRx(portHandler,
                                     dynamixel_id,
                                     ADDRESS[ACTUATOR_SETTING[dynamixel_id]['name']]['TORQUE_ENABLE'],
                                     True)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        quit()
    else:
        print("ID: %d Torque enabled. Ready to get & set Position." % dynamixel_id)


# ---------------------------------- Functions to communicate with DYNAMIXEL ----------------------------------
def set_position_callback(data):
    print("ID %s - Goal Position: %s, Velocity: MAX" % (data.id, data.position))
    dxl_comm_result, dxl_error =\
        packetHandler.write4ByteTxRx(portHandler,
                                     data.id,
                                     ADDRESS[ACTUATOR_SETTING[data.id]['name']]['GOAL_POSITION'],
                                     data.position)


def set_trajectory_callback(data):
    def deg2dynamixel_int(deg):
        return int(deg / 300 * 1024 + 0.5) + 511
    print("ID %s - Goal Position: %s, Velocity: %s" % (data.id, data.position, data.velocity))
    position = deg2dynamixel_int(data.position + ACTUATOR_SETTING[data.id]['initial_position_DGR'])
    data_packet = [DXL_LOBYTE(DXL_LOWORD(position)),
                   DXL_HIBYTE(DXL_LOWORD(position)),
                   DXL_LOBYTE(DXL_LOWORD(data.velocity)),
                   DXL_HIBYTE(DXL_LOWORD(data.velocity))]
    dxl_comm_result, dxl_error =\
        packetHandler.writeTxRx(portHandler,
                                data.id,
                                ADDRESS[ACTUATOR_SETTING[data.id]['name']]['GOAL_POSITION'],
                                4,
                                data_packet)


def get_current_position(req):
    dxl_present_position, dxl_comm_result, dxl_error =\
        packetHandler.read4ByteTxRx(portHandler,
                                    req.id,
                                    ADDRESS[ACTUATOR_SETTING[req.id]['name']]['PRESENT_POSITION'])
    print("Current Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position


def dynamixel_communicator():
    rospy.Subscriber('/set_position', SetPosition, set_position_callback, queue_size=10)
    rospy.Subscriber('/set_trajectory', SetTrajectory, set_trajectory_callback, queue_size=10)
    rospy.Service('/get_current_position', GetPosition, get_current_position)
    rospy.spin()


# ---------------------------------- Main function ----------------------------------
def main():
    open_serial_port()
    set_baudrate()
    for dynamixel_id in ACTUATOR_SETTING.keys():
        enable_torque(dynamixel_id)

    rospy.init_node('dynamixel_communicator')
    dynamixel_communicator()


if __name__ == '__main__':
    main()
