#!/usr/bin/env python

import pygame
from pygame.locals import *
import rospy
from geometry_msgs.msg import Point

pygame.init()
screen = pygame.display.set_mode((1024, 500))
clock = pygame.time.Clock()
rate = 50


def main():
    pub = rospy.Publisher('/pygame_pose', Point, queue_size=10)
    rospy.init_node('pygame_pose_pub')
    msg = Point()
    msg.y = 0
    msg.z = 0
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
        print(pygame.mouse.get_pos())
        msg.x = pygame.mouse.get_pos()[0]
        pub.publish(msg)
        clock.tick(rate)


if __name__ == '__main__':
    main()
