#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data

# from src.pybullet_debuger import pybulletDebug
# from src.kinematic_model import robotKinematics
# from src.gaitPlanner import trotGait
import logging

logger = logging.getLogger()
logger.setLevel(logging.INFO)

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.8)

p.setRealTimeSimulation(1)

while True:
    lastTime = time.time()
    #     pos , orn , L , angle , Lrot , T = pybulletDebug.cam_and_robotstates(boxId)
    #     #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    #     bodytoFeet = trot.loop(L , angle , Lrot , T , offset , bodytoFeet0)

    # #####################################################################################
    # #####   kinematics Model: Input body orientation, deviation and foot position    ####
    # #####   and get the angles, neccesary to reach that position, for every joint    ####
    #     FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn , pos , bodytoFeet)

    #     #move movable joints
    #     for i in range(0, footFR_index):
    #         p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    #     for i in range(footFR_index + 1, footFL_index):
    #         p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    #     for i in range(footFL_index + 1, footBR_index):
    #         p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    #     for i in range(footBR_index + 1, footBL_index):
    #         p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])

    logger.info(time.time() - lastTime)

p.disconnect()
