#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Caleb Hyde
@date: September, 2020
"""

import numpy as np
import pybullet as pb
import time
import logging
import sys

from src.pybullet_debugger import setup_pybullet, pybulletDebug
from src.model.kinematics import Quadruped
from src.model.gaitPlanner import trotGait
from src.model.geometrics import init_robot
from src.model import servo
from src.utils import run

ALPHA = 0.1
POSITION = [0, 0, -0.2]


def permute_ground(gnd, now):
    force = [ALPHA * np.sin(now), ALPHA * np.cos(now), 0]
    orientation = pb.getQuaternionFromEuler(force)
    # logging.debug(orientation)

    pb.resetBasePositionAndOrientation(
        bodyUniqueId=gnd,
        ornObj=orientation,
        posObj=POSITION,
    )


# foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
Xdist = 0.20
Ydist = 0.15
height = 0.15
B2F0 = np.array(
    # body frame to foot frame vector
    [
        [Xdist / 2, -Ydist / 2, -height],
        [Xdist / 2, Ydist / 2, -height],
        [-Xdist / 2, -Ydist / 2, -height],
        [-Xdist / 2, Ydist / 2, -height],
    ]
)
stance = np.array([0.5, 0.0, 0.0, 0.5])


def iterate(instance, robot, robotID, control, B2F0, stance, ground):
    now = time.time()
    position, orientation, velocity, angle, rotation, dT = instance.update(robotID)

    # calculates the feet coord for gait, defining length of the step and direction
    # (0º -> forward; 180º -> backward)
    # bodytoFeet = B2F0
    bodytoFeet = control.loop(velocity, angle, rotation, dT, stance, B2F0)

    # kinematics Model: Input body orientation, deviation and foot position
    # and get the angles, neccesary to reach that position, for every joint
    # FR_angles, FL_angles, BR_angles, BL_angles, bodyToFeet = robot.solve(
    pose = robot.solve(orientation, position, bodytoFeet)

    # move movable joints
    # angles = [*FR_angles, *FL_angles, *BR_angles, *BL_angles]
    angles = pose.flatten()

    permute_ground(ground, now)

    # TODO: something something jointIds
    # NOTE: this is not just range(12)
    indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for i, angle in enumerate(angles):
        pb.setJointMotorControl2(robotID, indices[i], pb.POSITION_CONTROL, angle)

    pb.stepSimulation()
    time.sleep(0.05)
    return instance, robot, robotID, control, B2F0, stance, ground


def go():
    try:
        boxId = setup_pybullet(grounded=False)
    except pb.error as e:
        logging.warning(e)
        sys.exit()
    pbdb = pybulletDebug(boxId)

    groundId = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[1, 1, 0.1])
    shapeID = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[1, 1, 0.1])
    ground = pb.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=groundId,
        basePosition=POSITION,
        baseVisualShapeIndex=shapeID,
    )

    robot = Quadruped()
    control = trotGait()
    B2F0, stance = init_robot()
    run(iterate, pbdb, robot, boxId, control, B2F0, stance, ground)
