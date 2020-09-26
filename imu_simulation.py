#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd

@author: Caleb Hyde
@date: September, 2020
"""

import logging
from os import environ
import numpy as np
import pybullet as pb
import time

from src.pybullet_debugger import setup_pybullet, pybulletDebug
from src.kinematics import Quadruped
from src.arduino import ArduinoSerial, get_ACM
from src.stabilization import stabilize


logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)

control = stabilize()
# period of time (in seconds) of every step
dT = 0.4
interval = 0.05
loopTime = time.time()

boxId = setup_pybullet()
pybulletDebug = pybulletDebug(boxId)
robot = Quadruped()


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


ARDUINO = False if environ.get("NO_ARDUINO") else True
if ARDUINO:
    arduino = get_ACM(tty="/dev/cu.usbmodem142301")
else:
    logger.info("Skipping Arduino setup!")

while True:
    loopTime = time.time()
    position, orientation, velocity, angle, rotation, dT = pybulletDebug.update(boxId)

    battery = 0
    Xacc = 0
    Yacc = 0
    realRoll = 0
    realPitch = 0
    if ARDUINO:
        lines = arduino.receive()

    for line in lines:
        logger.info(f"Received {line}")
        if "BNO055" in line:
            try:
                header, message = line.split("#")
                # NOTE that x,y,z are not the same order
                z, y, x, ax, ay, az = [float(s) for s in message.split(" ")]
                orientation = np.array([np.deg2rad(x), np.deg2rad(y), np.deg2rad(-z)])
            except ValueError as e:
                logger.debug(e)
            except Exception as e:
                logger.error(e)
                logger.warning(f"Failed to parse arduino: {line}")

    logger.warning(orientation)
    # pidx, pidy, pidox, pidoy = control.centerPoint(realPitch, realRoll)
    # position[0] = pidx
    # position[1] = pidy
    # orientation[0] = pidox
    # orientation[1] = pidoy
    # logger.debug(f"{loopTime}, {realRoll}, {realPitch}")

    # kinematics Model: Input body orientation, deviation and foot position
    # and get the angles, neccesary to reach that position, for every joint
    # FR_angles, FL_angles, BR_angles, BL_angles, bodyToFeet = robot.solve(
    pose = robot.solve(orientation, position, B2F0)

    # move movable joints
    # angles = [*FR_angles, *FL_angles, *BR_angles, *BL_angles]
    angles = pose.flatten()

    # TODO: something something jointIds
    indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for i, angle in enumerate(angles):
        pb.setJointMotorControl2(boxId, indices[i], pb.POSITION_CONTROL, angle)

    pb.stepSimulation()
    time.sleep(interval)

pb.disconnect()
