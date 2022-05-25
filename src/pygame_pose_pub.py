#!/usr/bin/env python

import pygame
from pygame.locals import *
import rospy
from TrackDesign2022.msg import *

pygame.init()
screen = pygame.display.set_mode((500, 500))
clock = pygame.time.Clock()
rate = 50


def main():
    pub = rospy.Publisher('/set_trajectory', SetTrajectory, queue_size=10)
    rospy.init_node('pygame_pose_pub')
    msg = SetTrajectory()

    # Set the motors which user wants to control
    control_motors = [2,12]

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
        msg.id = 1
        msg.position = -(pygame.mouse.get_pos()[0] - 250) / 250 * 90
        msg.velocity = 60
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 2
        msg.position = -(pygame.mouse.get_pos()[0]) / 500 * 90
        msg.velocity = 60
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 3
        msg.position = (pygame.mouse.get_pos()[0] - 250) / 250 * 150
        msg.velocity = 400
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 4
        msg.position = (pygame.mouse.get_pos()[0] - 500) / 500 * 90
        msg.velocity = 200
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 5
        msg.position = (pygame.mouse.get_pos()[0] - 250) / 250 * 150
        msg.velocity = 400
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 6
        msg.position = -(pygame.mouse.get_pos()[0] - 250) / 250 * 90
        msg.velocity = 400
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 11
        msg.position = (pygame.mouse.get_pos()[0] - 250) / 250 * 90
        msg.velocity = 60
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 12
        msg.position = (pygame.mouse.get_pos()[0]) / 500 * 90
        msg.velocity = 60
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 13
        msg.position = (pygame.mouse.get_pos()[0] - 250) / 250 * 150
        msg.velocity = 400
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 14
        msg.position = -(pygame.mouse.get_pos()[0] - 500) / 500 * 90
        msg.velocity = 200
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 15
        msg.position = (pygame.mouse.get_pos()[0] - 250) / 250 * 150
        msg.velocity = 400
        if msg.id in control_motors:
            pub.publish(msg)

        msg.id = 16
        msg.position = (pygame.mouse.get_pos()[0] - 250) / 250 * 90
        msg.velocity = 400
        if msg.id in control_motors:
            pub.publish(msg)

        clock.tick(rate)


if __name__ == '__main__':
    main()
