#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on September 22, 2020

@author: Caleb Hyde
"""
import time
import asyncio
import logging
import pybullet as pb
import numpy as np
import time
import pybullet_data
from functools import partial
from src.pybullet_debugger import pybulletDebug
from src.kinematics import Quadruped
from src.gaitPlanner import trotGait
from src.web.app import server

# from src.simulation import run as run_sim


async def run(instance):
    """Loop the pybullet simulation indefinitely"""
    while True:
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
        ) = robotKinematics.solve(pos, orn, bodytoFeet)

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


if __name__ == "__main__":

    physicsClient = pb.connect(pb.GUI)  # or pb.DIRECT for non-graphical version
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.setGravity(0, 0, -9.8)

    INTERVAL = 0.05

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
    trot = trotGait()

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

    pb.setRealTimeSimulation(1)

    # joystick = setup_joystick_async()
    TASKS = [
        # partial(read_joystick, joystick),
        partial(run, instance),
        server,
    ]
    app(TASKS)