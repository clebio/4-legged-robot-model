#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 22:15:21 2020

@author: linux-asd

@author: Caleb Hyde
@date: September, 2020
"""
import pybullet as pb
import time
import numpy as np
import sys
from typing import Tuple
import pybullet_data


def networking():
    """Run pybullet server on another computer

    A beefy gaming rig, for instance.
    https://github.com/bulletphysics/bullet3/blob/master/docs/pybullet_quickstart_guide/PyBulletQuickstartGuide.md.html#connect-using-direct-gui

    This method is just TODO code, not useful right now.
    """
    pybullet.connect(pybullet.DIRECT)
    pybullet.connect(pybullet.GUI, options="--opengl2")
    pybullet.connect(pybullet.SHARED_MEMORY, 1234)
    pybullet.connect(pybullet.UDP, "192.168.0.1")
    pybullet.connect(pybullet.UDP, "localhost", 1234)
    pybullet.connect(pybullet.TCP, "localhost", 6667)


def setup_pybullet(model="4leggedRobot.urdf", realtime=1, grounded=True):
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.setRealTimeSimulation(realtime)
    robot = pb.loadURDF(model, basePosition=[0, 0, 0.2])
    if grounded:
        ground = pb.loadURDF("plane.urdf")
        return ground, robot
    return robot


class pybulletDebug:
    UpdateTuple = Tuple[np.ndarray, np.ndarray, float, float, float, float]

    def __init__(self, baseObject):
        # Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot)
        self.base = baseObject
        self.cyaw = 90
        self.cpitch = -7
        self.cdist = 0.66

        self.xId = pb.addUserDebugParameter("x", -0.10, 0.10, 0.0)
        self.yId = pb.addUserDebugParameter("y", -0.10, 0.10, 0.0)
        self.zId = pb.addUserDebugParameter("z", -0.10, 0.10, 0.0)
        self.rollId = pb.addUserDebugParameter("roll", -np.pi / 4, np.pi / 4, 0.01)
        self.pitchId = pb.addUserDebugParameter("pitch", -np.pi / 4, np.pi / 4, 0.01)
        self.yawId = pb.addUserDebugParameter("yaw", -np.pi / 4, np.pi / 4, 0.01)
        self.LId = pb.addUserDebugParameter("L", -0.5, 1.5, 0.0)
        self.LrotId = pb.addUserDebugParameter("Lrot", -1.5, 1.5, 0.0)
        self.angleId = pb.addUserDebugParameter("angleWalk", -180.0, 180.0, 0.0)
        self.periodId = pb.addUserDebugParameter("stepPeriod", 0.1, 3.0, 0.78)

    def update(self, boxId) -> UpdateTuple:
        """Update pybullet simulation

        Returns:
            position
            orientation
            stride length
            angle
            stride rotation
            step period
        """
        keys = pb.getKeyboardEvents()
        # Keys to change camera
        if keys.get(100):  # D
            self.cyaw += 1
        if keys.get(97):  # A
            self.cyaw -= 1
        if keys.get(99):  # C
            self.cpitch += 1
        if keys.get(102):  # F
            self.cpitch -= 1
        if keys.get(122):  # Z
            self.cdist += 0.01
        if keys.get(120):  # X
            self.cdist -= 0.01

        if keys.get(27):  # ESC
            pb.disconnect()
            sys.exit()

        # TODO: doesn't work
        # if keys.get(114):  # R (reset)
        #     # pb.resetBaseVelocity(self.base, np.zeros(3), np.zeros(4))
        #     pb.resetBasePositionAndOrientation(
        #         self.base, [0, 0, 0.2], np.array([*orn, 0.0])
        #     )

        pos = np.array(
            [
                pb.readUserDebugParameter(self.xId),
                pb.readUserDebugParameter(self.yId),
                pb.readUserDebugParameter(self.zId),
            ]
        )
        orn = np.array(
            [
                pb.readUserDebugParameter(self.rollId),
                pb.readUserDebugParameter(self.pitchId),
                pb.readUserDebugParameter(self.yawId),
            ]
        )
        velocity = pb.readUserDebugParameter(self.LId)
        rotation = pb.readUserDebugParameter(self.LrotId)
        angle = pb.readUserDebugParameter(self.angleId)
        period = pb.readUserDebugParameter(self.periodId)

        cubePos, _ = pb.getBasePositionAndOrientation(boxId)

        pb.resetDebugVisualizerCamera(
            cameraDistance=self.cdist,
            cameraYaw=self.cyaw,
            cameraPitch=self.cpitch,
            cameraTargetPosition=cubePos,
        )
        return pos, orn, velocity, angle, rotation, period
