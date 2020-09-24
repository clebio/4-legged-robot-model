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


from src.simulation import setup_sim, run as run_sim

INTERVAL = 0.05


async def run():
    """Loop the pybullet simulation indefinitely"""
    instance = pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.setGravity(0, 0, -9.8)
    pb.setRealTimeSimulation(1)
    # pb.connect(pb.GRAPHICS_CLIENT)
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
        ) = robotKinematics.solve(orn, pos, bodytoFeet)

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
    # joystick = setup_joystick_async()
    TASKS = [
        # partial(read_joystick, joystick),
        run,
    ]
    app(TASKS)