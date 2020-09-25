#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 19:51:40 2020

@author: linux-asd
"""

import pybullet as pb
import numpy as np
import time
import pybullet_data

from src.pybullet_debugger import pybulletDebug
from src.kinematics import Quadruped
from src.gaitPlanner import trotGait

physicsClient = pb.connect(pb.GUI)  # or pb.DIRECT for non-graphical version
pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
pb.setGravity(0, 0, -9.8)

pb.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.2]
boxId = pb.loadURDF("4leggedRobot.urdf", cubeStartPos)

# jointIds = []
# paramIds = []
# for j in range(pb.getNumJoints(boxId)):
#     # pb.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
#     info = pb.getJointInfo(boxId, j)
#     # print(info)
#     jointName = info[1]
#     jointType = info[2]
#     jointIds.append(j)

# footFR_index = 3
# footFL_index = 7
# footBR_index = 11
# footBL_index = 15

pybulletDebug = pybulletDebug()
robot = Quadruped()
trot = trotGait()

# defines the offset between each foot step in this order (FR,FL,BR,BL)
offset = np.array([0.5, 0.0, 0.0, 0.5])

pb.setRealTimeSimulation(1)

while True:
    timeNow = time.time()
    pos, orn, L, angle, Lrot, dT = pybulletDebug.cam_and_robotstates(boxId)
    # calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    bodytoFeet = trot.loop(L, angle, Lrot, dT, offset)

    # kinematics Model: Input body orientation, deviation and foot position
    # and get the angles, neccesary to reach that position, for every joint
    (
        FR_angles,
        FL_angles,
        BR_angles,
        BL_angles,
    ) = robot.solve(orn, pos, bodytoFeet)

    # move movable joints
    angles = [*FR_angles, *FL_angles, *BR_angles, *BL_angles]

    # TODO: something something jointIds
    indices = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    for i, angle in enumerate(angles):
        pb.setJointMotorControl2(boxId, indices[i], pb.POSITION_CONTROL, angle)
    pb.stepSimulation()

    lastTime = time.time()
    loopTime = lastTime - timeNow
    time.sleep(0.05)
pb.disconnect()
