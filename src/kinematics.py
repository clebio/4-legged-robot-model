#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kinematics Model: Input body orientation, deviation and foot position
and get the angles, neccesary to reach that position, for every joint

Created on Thu Feb 27 15:21:52 2020

https://hackaday.io/project/171456-diy-hobby-servos-quadruped-robot/log/177488-lets-talk-about-the-kinematic-model

@author: miguel-asd
"""
import numpy as np
from src import geometrics as geo
import logging


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

p2 = lambda x: pow(x, 2)


def checkdomain(D):
    if D > 1 or D < -1:
        logging.debug("____OUT OF DOMAIN____")
        return np.sign(D) * 0.99
    else:
        return D


# this is based on this paper:
# "https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot"
"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/____________  x       "
"""


def _solve(coordinates, coxa, femur, tibia, orientation=1):
    """IK equations now written in pybullet frame"""

    x0, x1, x2 = coordinates
    _c1c2 = np.sqrt(x1 ** 2 + (-x2) ** 2 - coxa ** 2)

    numerator = p2(x1) + p2(-x2) - p2(coxa) + p2(-x0) - p2(femur) - p2(tibia)
    D = numerator / (2 * tibia * femur)
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1 - D ** 2), D)
    theta = -np.arctan2(x2, x1) - np.arctan2(_c1c2, orientation * coxa)

    alpha = np.arctan2(-x0, _c1c2) - np.arctan2(
        tibia * np.sin(gamma), femur + tibia * np.cos(gamma)
    )
    angles = np.array([-theta, alpha, gamma])
    return angles


class Quadruped:
    def __init__(
        self,
        Xdist=0.193,
        Ydist=0.11,
        W=0.077,
        height=0.15,
        coxa=0.05,
        femur=0.10,
        tibia=0.10,
    ):
        """initial robot position"""

        # self.targetAngs = target_angles
        """in meter"""
        self.Xdist = Xdist  # length of robot joints
        self.Ydist = Ydist
        self.height = height
        self.W = W  # width of robot joints
        self.coxa = coxa  # coxa length
        self.femur = femur  # femur length
        self.tibia = tibia  # tibia length

        """initial foot position"""
        # foot separation (0.182 -> tetta=0) and distance to floor
        # body frame to coxa frame vector
        self.bodytoFR0 = np.array([self.Xdist / 2, -self.W / 2, 0])
        self.bodytoFL0 = np.array([self.Xdist / 2, self.W / 2, 0])
        self.bodytoBR0 = np.array([-self.Xdist / 2, -self.W / 2, 0])
        self.bodytoBL0 = np.array([-self.Xdist / 2, self.W / 2, 0])
        # # body frame to foot frame vector
        # self.bodytoFR4 = np.array([self.Xdist / 2, -self.Ydist / 2, -self.height])
        # self.bodytoFL4 = np.array([self.Xdist / 2, self.Ydist / 2, -self.height])
        # self.bodytoBR4 = np.array([-self.Xdist / 2, -self.Ydist / 2, -self.height])
        # self.bodytoBL4 = np.array([-self.Xdist / 2, self.Ydist / 2, -self.height])

    def solve(self, orn: np.ndarray, pos: np.ndarray, bodytoFeet: np.array):
        """Solve inverse kinematics

        Input body orientation, deviation and foot position and get the angles neccesary to reach that position, for every joint"""
        bodytoFR4 = np.asarray([bodytoFeet[0, 0], bodytoFeet[0, 1], bodytoFeet[0, 2]])
        bodytoFL4 = np.asarray([bodytoFeet[1, 0], bodytoFeet[1, 1], bodytoFeet[1, 2]])
        bodytoBR4 = np.asarray([bodytoFeet[2, 0], bodytoFeet[2, 1], bodytoFeet[2, 2]])
        bodytoBL4 = np.asarray([bodytoFeet[3, 0], bodytoFeet[3, 1], bodytoFeet[3, 2]])

        """defines 4 vertices which rotates with the body"""
        _bodytoFR0 = geo.transform(self.bodytoFR0, orn, pos)
        _bodytoFL0 = geo.transform(self.bodytoFL0, orn, pos)
        _bodytoBR0 = geo.transform(self.bodytoBR0, orn, pos)
        _bodytoBL0 = geo.transform(self.bodytoBL0, orn, pos)
        """defines coxa_frame to foot_frame leg vector neccesary for IK"""
        FRcoord = bodytoFR4 - _bodytoFR0
        FLcoord = bodytoFL4 - _bodytoFL0
        BRcoord = bodytoBR4 - _bodytoBR0
        BLcoord = bodytoBL4 - _bodytoBL0
        """undo transformation of leg vector to keep feet still"""
        undoOrn = -orn
        undoPos = -pos
        _FRcoord = geo.transform(FRcoord, undoOrn, undoPos)
        _FLcoord = geo.transform(FLcoord, undoOrn, undoPos)
        _BRcoord = geo.transform(BRcoord, undoOrn, undoPos)
        _BLcoord = geo.transform(BLcoord, undoOrn, undoPos)

        FR_angles = _solve(_FRcoord, self.coxa, self.femur, self.tibia, -1)
        FL_angles = _solve(_FLcoord, self.coxa, self.femur, self.tibia, 1)
        BR_angles = _solve(_BRcoord, self.coxa, self.femur, self.tibia, -1)
        BL_angles = _solve(_BLcoord, self.coxa, self.femur, self.tibia, 1)

        _bodytofeetFR = _bodytoFR0 + _FRcoord
        _bodytofeetFL = _bodytoFL0 + _FLcoord
        _bodytofeetBR = _bodytoBR0 + _BRcoord
        _bodytofeetBL = _bodytoBL0 + _BLcoord

        pose = np.array([FR_angles, FL_angles, BR_angles, BL_angles])
        # _bodytofeet = np.array(
        #     [_bodytofeetFR, _bodytofeetFL, _bodytofeetBR, _bodytofeetBL]
        # )

        # return FR_angles, FL_angles, BR_angles, BL_angles, _bodytofeet
        return pose
