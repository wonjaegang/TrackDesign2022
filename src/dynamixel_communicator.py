#!/usr/bin/env python

import rospy
from dynamixel_sdk import *
from TrackDesign2022.srv import *
from TrackDesign2022.msg import *

# Control table address
ADDRESS = {'AX18A': {'TORQUE_ENABLE': 24,
                     'GOAL_POSITION': 30,
                     'PRESENT_POSITION': 36,
                     'COMPLIANCE_SLOPE': 28,
                     'PUNCH': 48},
           'XM430': {'TORQUE_ENABLE': 64,
                     'GOAL_POSITION': 112,             # GOAL_POSITION address is 116. revised for velocity control.
                     'PRESENT_POSITION': 132}}

# Protocol version
PROTOCOL_VERSION = 1.0             # protocol of AX-18A. Depends on motor.

# Default setting
BAUD_RATE = 1000000                 # Dynamixel default baudrate : 57600
DEVICE_NAME = '/dev/ttyUSB0'        # Check which port is being used on your controller

# Actuator initial setting. Use ID as keys.
ACTUATOR_SETTING = {1: {'name': 'XM430',
                        'initial_position_DGR': 0},
                    2: {'name': 'XM430',
                        'initial_position_DGR': 0},
                    3: {'name': 'AX18A',
                        'initial_position_DGR': 0,
                        'initial_CW_slope': 254,
                        'initial_CCW_slope': 254,
                        'initial_punch': 0},
                    4: {'name': 'AX18A',
                        'initial_position_DGR': 90,
                        'initial_CW_slope': 90,
                        'initial_CCW_slope': 90,
                        'initial_punch': 100},
                    5: {'name': 'AX18A',
                        'initial_position_DGR': 0,
                        'initial_CW_slope': 254,
                        'initial_CCW_slope': 254,
                        'initial_punch': 0},
                    6: {'name': 'AX18A',
                        'initial_position_DGR': 0,
                        'initial_CW_slope': 254,
                        'initial_CCW_slope': 254,
                        'initial_punch': 0},
                    11: {'name': 'XM430',
                         'initial_position_DGR': 0},
                    12: {'name': 'XM430',
                         'initial_position_DGR': 0},
                    13: {'name': 'AX18A',
                         'initial_position_DGR': 0,
                         'initial_CW_slope': 254,
                         'initial_CCW_slope': 254,
                         'initial_punch': 0},
                    14: {'name': 'AX18A',
                         'initial_position_DGR': -90,
                         'initial_CW_slope': 90,
                         'initial_CCW_slope': 90,
                         'initial_punch': 100},
                    15: {'name': 'AX18A',
                         'initial_position_DGR': 0,
                         'initial_CW_slope': 254,
                         'initial_CCW_slope': 254,
                         'initial_punch': 0},
                    16: {'name': 'AX18A',
                         'initial_position_DGR': 0,
                         'initial_CW_slope': 254,
                         'initial_CCW_slope': 254,
                         'initial_punch': 0},
                    21: {'name': 'AX18A',
                         'initial_position_DGR': 0,
                         'initial_CW_slope': 254,
                         'initial_CCW_slope': 254,
                         'initial_punch': 0},
                    22: {'name': 'AX18A',
                         'initial_position_DGR': 0,
                         'initial_CW_slope': 254,
                         'initial_CCW_slope': 254,
                         'initial_punch': 0}
                    }

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


def set_initial_value(dynamixel_id):
    if ACTUATOR_SETTING[dynamixel_id]['name'] == 'AX18A':
        packetHandler.write1ByteTxRx(portHandler,
                                     dynamixel_id,
                                     ADDRESS[ACTUATOR_SETTING[dynamixel_id]['name']]['COMPLIANCE_SLOPE'],
                                     ACTUATOR_SETTING[dynamixel_id]['initial_CW_slope'])
        packetHandler.write1ByteTxRx(portHandler,
                                     dynamixel_id,
                                     ADDRESS[ACTUATOR_SETTING[dynamixel_id]['name']]['COMPLIANCE_SLOPE'] + 1,
                                     ACTUATOR_SETTING[dynamixel_id]['initial_CCW_slope'])
        packetHandler.write2ByteTxRx(portHandler,
                                     dynamixel_id,
                                     ADDRESS[ACTUATOR_SETTING[dynamixel_id]['name']]['PUNCH'],
                                     ACTUATOR_SETTING[dynamixel_id]['initial_punch'])
    print("ID: %d Value initialized." % dynamixel_id)


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
    packetHandler.write4ByteTxRx(portHandler,
                                 data.id,
                                 ADDRESS[ACTUATOR_SETTING[data.id]['name']]['GOAL_POSITION'],
                                 data.position)


def set_trajectory_callback(data_array):
    print("-" * 50)
    for data in data_array.data:
        if ACTUATOR_SETTING[data.id]['name'] == 'AX18A':
            # Convert degree -> DYNAMIXEL integer(AX18A: 0 ~ 1023)
            def deg2dynamixel_int(deg):
                return int(deg / 300 * 1024 + 0.5) + 511

            def dps2dynamixel_int(dps):
                return int(dps / 6 / 0.111 + 0.5)

            position_int = deg2dynamixel_int(data.position + ACTUATOR_SETTING[data.id]['initial_position_DGR'])
            velocity_int = dps2dynamixel_int(data.velocity)
            print("ID %s - Goal Position(INT): %d, Velocity(INT): %s" % (data.id, position_int, velocity_int))
            data_packet = [DXL_LOBYTE(DXL_LOWORD(position_int)),
                           DXL_HIBYTE(DXL_LOWORD(position_int)),
                           DXL_LOBYTE(DXL_LOWORD(velocity_int)),
                           DXL_HIBYTE(DXL_LOWORD(velocity_int))]

            packetHandler.writeTxRx(portHandler,
                                    data.id,
                                    ADDRESS[ACTUATOR_SETTING[data.id]['name']]['GOAL_POSITION'],
                                    4,
                                    data_packet)

        if ACTUATOR_SETTING[data.id]['name'] == 'XM430':
            # Convert degree -> DYNAMIXEL integer(XM430: 0 ~ 4095)
            def deg2dynamixel_int(deg):
                return int(deg / 360 * 4096 + 0.5) + 2047

            def dps2dynamixel_int(dps):
                return int(dps / 6 / 0.229 + 0.5)

            position_int = deg2dynamixel_int(data.position + ACTUATOR_SETTING[data.id]['initial_position_DGR'])
            velocity_int = dps2dynamixel_int(data.velocity)
            print("ID %s - Goal Position(INT): %d, Velocity(INT): %s" % (data.id, position_int, velocity_int))
            data_packet = [DXL_LOBYTE(DXL_LOWORD(velocity_int)),
                           DXL_HIBYTE(DXL_LOWORD(velocity_int)),
                           DXL_LOBYTE(DXL_HIWORD(velocity_int)),
                           DXL_HIBYTE(DXL_HIWORD(velocity_int)),
                           DXL_LOBYTE(DXL_LOWORD(position_int)),
                           DXL_HIBYTE(DXL_LOWORD(position_int)),
                           DXL_LOBYTE(DXL_HIWORD(position_int)),
                           DXL_HIBYTE(DXL_HIWORD(position_int))]
            packetHandler.writeTxRx(portHandler,
                                    data.id,
                                    ADDRESS[ACTUATOR_SETTING[data.id]['name']]['GOAL_POSITION'],
                                    8,
                                    data_packet)

def get_current_position(req):
    dxl_present_position, dxl_comm_result, dxl_error =\
        packetHandler.read4ByteTxRx(portHandler,
                                    req.id,
                                    ADDRESS[ACTUATOR_SETTING[req.id]['name']]['PRESENT_POSITION'])
    print("Current Position(INT) of ID %s = %s" % (req.id, dxl_present_position))
    if ACTUATOR_SETTING[req.id]['name'] == 'AX18A':
        if dxl_present_position < 0:
            dxl_present_position = 0
        if dxl_present_position > 1023:
            dxl_present_position = 1023
        return (dxl_present_position - 511) / 1024 * 300 - ACTUATOR_SETTING[req.id]['initial_position_DGR']
    else:
        if dxl_present_position < 0:
            dxl_present_position = 0
        if dxl_present_position > 4095:
            dxl_present_position = 4095
        return (dxl_present_position - 2047) / 4096 * 360 - ACTUATOR_SETTING[req.id]['initial_position_DGR']


def dynamixel_communicator():
    rospy.Subscriber('/set_position', SetPosition, set_position_callback, queue_size=1)
    rospy.Subscriber('/set_trajectory', SetTrajectoryArray, set_trajectory_callback, queue_size=1)
    rospy.Service('/get_current_position', GetPosition, get_current_position)
    rospy.spin()


# ---------------------------------- Main function ----------------------------------
def main():
    open_serial_port()
    set_baudrate()
    for dynamixel_id in ACTUATOR_SETTING.keys():
        set_initial_value(dynamixel_id)
        enable_torque(dynamixel_id)

    rospy.init_node('dynamixel_communicator')
    dynamixel_communicator()


if __name__ == '__main__':
    main()
