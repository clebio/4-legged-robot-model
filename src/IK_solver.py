#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 18:20:45 2020

@author: miguel-asd
"""
import numpy as np
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def checkdomain(D):
    if D > 1 or D < -1:
        logging.debug("____OUT OF DOMAIN____")
        if D > 1:
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
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
# IK equations now written in pybullet frame.
def solve_R(coord, coxa, femur, tibia):
    D = (
        coord[1] ** 2
        + (-coord[2]) ** 2
        - coxa ** 2
        + (-coord[0]) ** 2
        - femur ** 2
        - tibia ** 2
    ) / (
        2 * tibia * femur
    )  # siempre <1
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1 - D ** 2), D)
    theta = -np.arctan2(coord[2], coord[1]) - np.arctan2(
        np.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2), -coxa
    )
    alpha = np.arctan2(
        -coord[0], np.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2)
    ) - np.arctan2(tibia * np.sin(gamma), femur + tibia * np.cos(gamma))
    angles = np.array([-theta, alpha, gamma])
    return angles


def solve_L(coord, coxa, femur, tibia):
    D = (
        coord[1] ** 2
        + (-coord[2]) ** 2
        - coxa ** 2
        + (-coord[0]) ** 2
        - femur ** 2
        - tibia ** 2
    ) / (
        2 * tibia * femur
    )  # siempre <1
    D = checkdomain(D)

    gamma = np.arctan2(-np.sqrt(1 - D ** 2), D)
    theta = -np.arctan2(coord[2], coord[1]) - np.arctan2(
        np.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2), coxa
    )
    alpha = np.arctan2(
        -coord[0], np.sqrt(coord[1] ** 2 + (-coord[2]) ** 2 - coxa ** 2)
    ) - np.arctan2(tibia * np.sin(gamma), femur + tibia * np.cos(gamma))
    angles = np.array([-theta, alpha, gamma])
    return angles

