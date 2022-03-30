#!/usr/bin/env python

import rospy
from dynamixel_sdk import *
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *

# Control table address
ADDR_TORQUE_ENABLE = 24                    # Address of AX-18A. Depends on motor.
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36

# Protocol version
PROTOCOL_VERSION = 1.0             # protocol of AX-18A. Depends on motor.

# Default setting
DXL_ID = 1                         # Dynamixel ID : 1
BAUDRATE = 1000000                 # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'        # Check which port is being used on your controller

TORQUE_ENABLE = 1                  # Value for enabling the torque
TORQUE_DISABLE = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE = 0     # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE = 1000  # Dynamixel would not move when the position value is out of movable range
DXL_MOVING_STATUS_THRESHOLD = 20   # Dynamixel moving status threshold

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
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        quit()
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        quit()
    else:
        print("Torque enabled. Ready to get & set Position.")


# ---------------------------------- Functions to communicate with DYNAMIXEL ----------------------------------
def set_goal_pos_callback(data):
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)


def set_goal_pos_speed_callback(data):
    print("Set Speed of ID %s = %s" % (data.id, data.position))
    temp_position = 512
    temp_speed = 300
    temp_data = [DXL_LOBYTE(DXL_LOWORD(temp_position)),
                 DXL_HIBYTE(DXL_LOWORD(temp_position)),
                 DXL_LOBYTE(DXL_LOWORD(temp_speed)),
                 DXL_HIBYTE(DXL_LOWORD(temp_speed))]
    dxl_comm_result, dxl_error = packetHandler.writeTxRx(portHandler, data.id, ADDR_GOAL_POSITION, 4, temp_data)


def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error =\
        packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position


def dynmixel_communicator():
    rospy.init_node('dynmixel_communicator')
    rospy.Subscriber('/set_position', SetPosition, set_goal_pos_callback)
    rospy.Subscriber('/set_position_and_speed', SetPosition, set_goal_pos_speed_callback)
    rospy.Service('get_position', GetPosition, get_present_pos)
    rospy.spin()


# ---------------------------------- Main function ----------------------------------
def main():
    open_serial_port()
    set_baudrate()
    enable_torque()

    dynmixel_communicator()


if __name__ == '__main__':
    main()
