#!/usr/bin/env python3
# -- coding: utf-8 --

import numpy as np

from pykin.kinematics import transform as tf
from pykin.robots.bimanual import Bimanual
from pykin.kinematics import jacobian as jac

file_path = 'src/TrackDesign2022/urdf/meta_arm.xacro'
robot = Bimanual(file_path, tf.Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]))

meta_arm = np.zeros(7)
robot.setup_link_name("base_link", "point")
#robot.setup_link_name("base", "left_wrist")

fk = robot.forward_kin(meta_arm)

J = {}
for arm in robot.arm_type:
    if robot.eef_name[arm]:
        J[arm] = jac.calc_jacobian(robot.desired_frames[arm], fk, len(np.zeros(7)))

print(J)