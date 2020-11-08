#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pybullet simulation

@author: Caleb Hyde
@date: September 22, 2020
"""
import asyncio
import time
import logging
import pybullet as pb
import numpy as np
import pybullet_data

from src.pybullet_debugger import pybulletDebug
from src.kinematics import Quadruped
from src.gaitPlanner import trotGait

INTERVAL = 0.05


async def setup_sim():
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.setGravity(0, 0, -9.8)
    pb.setRealTimeSimulation(1)
    while pb.isConnected():
        pb.stepSimulation()
        await asyncio.sleep(INTERVAL)


async def run():
    """Loop the pybullet simulation indefinitely"""
    physicsClient = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.setGravity(0, 0, -9.8)
    pb.setRealTimeSimulation(1)
    pb.connect(pb.GRAPHICS_CLIENT)
    # pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    cubeStartPos = [0, 0, 0.2]
    pb.loadURDF("plane.urdf")
    boxId = pb.loadURDF("4leggedRobot.urdf", cubeStartPos)

    jointIds = []
    paramIds = []

    for j in range(pb.getNumJoints(boxId)):
        # pb.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
        info = pb.getJointInfo(boxId, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        jointIds.append(j)

    footFR_index = 3
    footFL_index = 7
    footBR_index = 11
    footBL_index = 15

    instance = pybulletDebug()
    robotKinematics = Quadruped()

    # foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
    Xdist = 0.20
    Ydist = 0.15
    height = 0.15
    # body frame to foot frame vector
    bodytoFeet0 = np.matrix(
        [
            [Xdist / 2, -Ydist / 2, -height],
            [Xdist / 2, Ydist / 2, -height],
            [-Xdist / 2, -Ydist / 2, -height],
            [-Xdist / 2, Ydist / 2, -height],
        ]
    )
    # defines the offset between each foot step in this order (FR,FL,BR,BL)
    offset = np.array([0.5, 0.0, 0.0, 0.5])

    trot = trotGait()

    while pb.isConnected():
        timeNow = time.time()
        pos, orn, L, angle, Lrot, dT = instance.cam_and_robotstates(boxId)
        # calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        bodytoFeet = trot.loop(L, angle, Lrot, dT, offset, bodytoFeet0)

        # kinematics Model: Input body orientation, deviation and foot position
        # and get the angles, neccesary to reach that position, for every joint
        (
            FR_angles,
            FL_angles,
            BR_angles,
            BL_angles,
        ) = robot.solve(orn, pos, bodytoFeet)

        angles = [*FR_angles, *FL_angles, *BR_angles, *BL_angles]

        # TODO: something something jointIds
        indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
        for i, angle in enumerate(angles):
            pb.setJointMotorControl2(boxId, indices[i], pb.POSITION_CONTROL, angle)
        pb.stepSimulation()

        lastTime = time.time()
        loopTime = lastTime - timeNow
        logging.debug(f"simulation at {loopTime}")
        await asyncio.sleep(INTERVAL)

    pb.disconnect()
