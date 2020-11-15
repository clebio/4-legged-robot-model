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
from src.model.gaitPlanner import trotGait
from src.model.stabilization import stabilize
from src.model.geometrics import init_robot
from src.utils import run


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def iterate_imu(instance, robot, robotID, control, b2f, stance, arduino, orientation):
    position, _, velocity, angle, rotation, dT = instance.update(robotID)
    lines = arduino.receive()
    for line in lines:
        logger.info(f"Received {line}")
        if "BNO055" in line or "LSM9DS1" in line:
            try:
                header, message = line.split("#")
                # NOTE that x,y,z are not the same order
                x, y, z, ax, ay, az = [float(s) for s in message.split(" ")]
                # orientation = np.array([np.deg2rad(x), np.deg2rad(y), np.deg2rad(-z)])
                orientation = np.array([x, y, z])
            except Exception as e:
                logger.warning(f"Failed to parse arduino: {line} ({e})")

    # b2f = control.loop(velocity, angle, rotation, dT, stance, b2f)
    pose = robot.solve(position, orientation, b2f)

    angles = pose.flatten()
    indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for i, angle in enumerate(angles):
        pb.setJointMotorControl2(robotID, indices[i], pb.POSITION_CONTROL, angle)

    return instance, robot, robotID, control, b2f, stance, arduino, orientation


def go():
    interval = 0.05
    _, boxId = setup_pybullet()  # model="cube_small.urdf")
    pbdb = pybulletDebug(boxId)
    robot = Quadruped()
    # control = stabilize()
    B2F0, stance = init_robot()
    control = trotGait(B2F0)

    DEVICE = "/dev/cu.usb*"  # MacOS laptop
    # DEVICE = "/dev/ttyACM*"  # Raspberry Pi
    arduino = get_ACM(tty=DEVICE)
    if not arduino:
        logging.warning("Expecting an Arduino to be connected (to provide IMU data)")
        return False
    run(iterate_imu, pbdb, robot, boxId, control, B2F0, stance, arduino, np.zeros(3))
