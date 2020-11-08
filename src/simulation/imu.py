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

from src.arduino import ArduinoSerial, get_ACM
from src.pybullet_debugger import setup_pybullet, pybulletDebug
from src.model.kinematics import Quadruped
from src.model.stabilization import stabilize
from src.model.geometrics import init_robot
from src.utils import run


logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)


def iterate_imu(debugger, robot, robotID, control, orientation, arduino, B2F0):
    position, _, velocity, angle, rotation, dT = debugger.update(robotID)
    if arduino:
        lines = arduino.receive()
        for line in lines:
            logger.info(f"Received {line}")
            if "BNO055" in line or "LSM9DS1" in line:
                try:
                    header, message = line.split("#")
                    # NOTE that x,y,z are not the same order
                    z, y, x, ax, ay, az = [float(s) for s in message.split(" ")]
                    orientation = np.array(
                        [np.deg2rad(x), np.deg2rad(y), np.deg2rad(-z)]
                    )
                except Exception as e:
                    logger.warning(f"Failed to parse arduino: {line} ({e})")

    logger.info(orientation)
    pose = robot.solve(orientation, position, B2F0)
    # logger.info(pose)
    angles = pose.flatten()
    indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for i, angle in enumerate(angles):
        pb.setJointMotorControl2(robotID, indices[i], pb.POSITION_CONTROL, angle)
    return (debugger, robot, robotID, control, orientation, arduino, B2F0)


def stand():
    interval = 0.05
    _, boxId = setup_pybullet()  # model="cube_small.urdf")
    pbdb = pybulletDebug(boxId)
    robot = Quadruped()
    control = stabilize()
    B2F0, stance = init_robot()

    DEVICE = "/dev/cu.usb*"  # MacOS laptop
    # DEVICE = "/dev/ttyACM*"  # Raspberry Pi
    arduino = get_ACM(tty=DEVICE)
    run(iterate_imu, pbdb, robot, boxId, control, np.zeros(3), arduino, B2F0)
