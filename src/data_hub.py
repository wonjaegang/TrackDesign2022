#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from oculus_sub import oculus_sub

def main():
    rospy.init_node('datahub', anonymous=True)
    oculus = oculus_sub()
    rospy.sleep(0.5)
    print(" Start!! ")
    
    while not rospy.is_shutdown():
        print(oculus.head_pose)
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
