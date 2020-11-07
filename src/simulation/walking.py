#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd

@author: Caleb Hyde
@date: September, 2020
"""

import numpy as np
import pybullet as pb

from src.pybullet_debugger import setup_pybullet, pybulletDebug
from src.model.kinematics import Quadruped
from src.model.gaitPlanner import trotGait
from src.model import servo
from src.model.geometrics import init_robot
from src.utils import run


def iterate_walk(instance, robot, robotID, walker, B2F0, stance):
    position, orientation, velocity, angle, rotation, dT = instance.update(robotID)

    # calculates the feet coord for gait, defining length of the step and direction
    # (0º -> forward; 180º -> backward)
    # bodytoFeet = B2F0
    bodytoFeet = walker.loop(velocity, angle, rotation, dT, stance, B2F0)

    # kinematics Model: Input body orientation, deviation and foot position
    # and get the angles, neccesary to reach that position, for every joint
    # FR_angles, FL_angles, BR_angles, BL_angles, bodyToFeet = robot.solve(
    pose = robot.solve(orientation, position, bodytoFeet)

    # move movable joints
    # angles = [*FR_angles, *FL_angles, *BR_angles, *BL_angles]
    angles = pose.flatten()

    # TODO: something something jointIds
    # NOTE: this is not just range(12)
    indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for i, angle in enumerate(angles):
        pb.setJointMotorControl2(robotID, indices[i], pb.POSITION_CONTROL, angle)
    return instance, robot, robotID, walker, B2F0, stance


def walk():
    _, boxId = setup_pybullet()
    pbdb = pybulletDebug(boxId)
    robot = Quadruped()
    trot = trotGait()
    B2F0, stance = init_robot()
    run(iterate_walk, pbdb, robot, boxId, trot, B2F0, stance)
    