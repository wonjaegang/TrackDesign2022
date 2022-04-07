#!/usr/bin/env python

import rospy
from dynamixel_sdk import *
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *
from geometry_msgs.msg import Point

# Control table address
ADDR_TORQUE_ENABLE = 24                    # Address of AX-18A. Depends on motor.
ADDR_GOAL_POSITION = 30                    # Position value : 0 ~ 1023, Velocity value : 0 ~ 1023
ADDR_PRESENT_POSITION = 36

# Protocol version
PROTOCOL_VERSION = 1.0             # protocol of AX-18A. Depends on motor.

# Default setting
DXL_ID = 1                         # Dynamixel ID : 1
BAUDRATE = 1000000                 # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'        # Check which port is being used on your controller

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


# ---------------------------------- Functions to initiate DYNAMIXEL ----------------------------------
def open_serial_port():
    try:
        portHandler.openPort()
        print("Succeeded to open the port, current port: %s" % DEVICENAME)
    except:
        print("Failed to open the port")
        quit()


def set_baudrate():
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate, current baudrate: %d" % BAUDRATE)
    except:
        print("Failed to change the baudrate")
        quit()


def enable_torque():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, True)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        quit()
    else:
        print("Torque enabled. Ready to get & set Position.")


# ---------------------------------- Functions to communicate with DYNAMIXEL ----------------------------------
def set_position_callback(data):
    print("ID %s - Goal Position: %s, Velocity: MAX" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)


def set_trajectory_callback(data):
    print("ID %s - Goal Position: %s, Velocity: %s" % (data.id, data.position, data.velocity))
    data_packet = [DXL_LOBYTE(DXL_LOWORD(data.position)),
                   DXL_HIBYTE(DXL_LOWORD(data.position)),
                   DXL_LOBYTE(DXL_LOWORD(data.velocity)),
                   DXL_HIBYTE(DXL_LOWORD(data.velocity))]
    dxl_comm_result, dxl_error = packetHandler.writeTxRx(portHandler, data.id, ADDR_GOAL_POSITION, 4, data_packet)


def get_current_position(req):
    dxl_present_position, dxl_comm_result, dxl_error =\
        packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Current Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position


def dynamixel_communicator():
    rospy.init_node('dynamixel_communicator')
    rospy.Subscriber('/set_position', SetPosition, set_position_callback, queue_size=10)
    rospy.Subscriber('/set_trajectory', SetTrajectory, set_trajectory_callback, queue_size=10)
    rospy.Service('/get_current_position', GetPosition, get_current_position)
    rospy.spin()


# ---------------------------------- Main function ----------------------------------
def main():
    open_serial_port()
    set_baudrate()
    enable_torque()

    dynamixel_communicator()


if __name__ == '__main__':
    main()
