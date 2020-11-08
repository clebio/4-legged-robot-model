#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on September 22, 2020

@author: Caleb Hyde
"""
import time
import asyncio
import logging
import numpy as np
from functools import partial
from src.model.kinematics import Quadruped
from src.model.gaitPlanner import trotGait
from src.model.geometrics import init_robot

# from src.web.app import server
from os import environ

# from src.simulation import run as run_sim

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

import pybullet as pb
import pybullet_data
from src.pybullet_debugger import setup_pybullet, pybulletDebug


async def run(instance, robot, robotID, walker, B2F0, stance):
    """Loop the pybullet simulation indefinitely"""
    INTERVAL = 0.05

    # robot properties
    """initial foot position"""
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

    while True:
        timeNow = time.time()
        pos, orn, L, angle, Lrot, dT = instance.update(robotID)
        # calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
        bodytoFeet = walker.loop(L, angle, Lrot, dT, offset, bodytoFeet0)

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
            pb.setJointMotorControl2(robotID, indices[i], pb.POSITION_CONTROL, angle)
        pb.stepSimulation()

        lastTime = time.time()
        loopTime = lastTime - timeNow
        logging.debug(f"simulation at {loopTime}")
        await asyncio.sleep(INTERVAL)
        # time.sleep(INTERVAL)


def app(task_list):
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    loop = asyncio.new_event_loop()

    tasks = []
    for task in task_list:
        tasks.append(loop.create_task(task()))

    try:
        loop.run_forever()
    except KeyboardInterrupt as ki:
        pass
    finally:
        for task in tasks:
            task.cancel()
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
        logging.info("Done with cleanup!")
        pb.disconnect()


def main():
    _, boxId = setup_pybullet()

    jointIds = []
    paramIds = []
    for j in range(pb.getNumJoints(boxId)):
        # pb.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
        info = pb.getJointInfo(boxId, j)
        # print(info)
        jointName = info[1]
        jointType = info[2]
        jointIds.append(j)

    instance = pybulletDebug(boxId)
    robot = Quadruped()
    trot = trotGait()
    pb.setRealTimeSimulation(1)
    B2F0, stance = init_robot()
    # run(instance, robot, boxId, trot, B2F0, stance)
    # joystick = setup_joystick_async()
    TASKS = [
        # partial(read_joystick, joystick),
        partial(run, instance, robot, boxId, trot, B2F0, stance),
        # server,
    ]
    app(TASKS)