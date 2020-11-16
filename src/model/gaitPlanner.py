#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 16:38:15 2020

@author: miguel-asd
"""
import time
import numpy as np

# defines the offset between each foot step in this order (FR,FL,BR,BL)
# offset between each foot step in this order (FR,FL,BR,BL)
_trot = np.array([0.5, 0.0, 0.0, 0.5])
_creep = np.array([0.0, 0.25, 0.75, 0.5])
_trot_rev = np.array([0.0, 0.5, 0.5, 0.0])
_stance = _trot


def f(n, k):
    """calculates binomial factor (n k) to build a parametrized bezier curve"""
    return np.math.factorial(n) / (np.math.factorial(k) * np.math.factorial(n - k))


def b(t, k, point, n=10):
    """Bezier curve, with n points"""
    n -= 1
    return point * f(n, k) * np.power(t, k) * np.power(1 - t, n - k)


class trotGait:
    """Gait planner to move all feet

    This trajectory planning is mostly based on:
        https://www.researchgate.net/publication/332374021_Leg_Trajectory_Planning_for_Quadruped_Robots_with_High-Speed_Trot_Gait
    """

    def __init__(self, b2f=np.zeros([4, 3])):
        self.bodytoFeet = b2f
        self.phi = 0.0
        self.phiStance = 0.0
        self.lastTime = 0.0
        self.alpha = 0.0
        self.s = False

    def calculateStance(self, phi_st, V, angle):
        """phi_st between [0,1), angle in degrees"""

        # cylindrical coordinates
        c = np.cos(np.deg2rad(angle))
        s = np.sin(np.deg2rad(angle))

        A = 0.0005
        halfStance = 0.05
        p_stance = halfStance * (1 - 2 * phi_st)

        stanceX = c * p_stance * np.abs(V)
        stanceY = -s * p_stance * np.abs(V)
        stanceZ = -A * np.cos(np.pi / (2 * halfStance) * p_stance)

        return stanceX, stanceY, stanceZ

    def calculateBezier_swing(self, phi_sw, V, angle):
        """phi between [0,1), angle in degrees

        Inputs:
            phi_sw
            V: velocity ("multiply all points by a velocity to make the trajectory wider")
            angle

        returns:
            swingX
            swingY
            swingZ
        https://hackaday.io/project/171456-diy-hobby-servos-quadruped-robot/log/178481-step-trajectory-and-gait-planner-from-mit-cheetah

        curve generator https://www.desmos.com/calculator/xlpbe9bgll
        """

        # cylindrical coordinates
        c = np.cos(np.deg2rad(angle))
        s = np.sin(np.deg2rad(angle))
        # if (phi >= 0.75 or phi < 0.25):
        #     self.s = True
        #     print('foot DOWN' , self.s , phi)

        # elif (phi <= 0.75 and phi > 0.25):
        #     self.s = False
        #     print('foot UP', self.s , phi)

        what_the_x = [-0.05, -0.06, -0.07, -0.07, 0.0, 0.0, 0.07, 0.07, 0.06, 0.05]
        what_the_y = [0.05, 0.06, 0.07, 0.07, 0.0, -0.0, -0.07, -0.07, -0.06, -0.05]
        what_the_z = [0.0, 0.0, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06, 0.0, 0.0]
        X = np.abs(V) * c * np.array(what_the_x)
        Y = np.abs(V) * s * np.array(what_the_y)
        Z = np.abs(V) * np.array(what_the_z)

        swingX = 0.0
        swingY = 0.0
        swingZ = 0.0

        assert len(what_the_x) == len(what_the_y) == len(what_the_z)
        curve_len = len(what_the_x)
        for i in range(curve_len):  # sum all terms of the curve
            swingX = swingX + b(phi_sw, i, X[i])
            swingY = swingY + b(phi_sw, i, Y[i])
            swingZ = swingZ + b(phi_sw, i, Z[i])

        return swingX, swingY, swingZ

    def stepTrajectory(self, phi, V, angle, Wrot, centerToFoot) -> np.ndarray:
        """phi belong [0,1), angles in degrees

        returns coord: np.array(3)
        """
        if phi >= 1:
            phi = phi - 1.0
        # step describes a circuference in order to rotate
        r = np.sqrt(centerToFoot[0] ** 2 + centerToFoot[1] ** 2)
        # radius of the ciscunscribed circle
        footAngle = np.arctan2(centerToFoot[1], centerToFoot[0])

        # As it is defined inside cylindrical coordinates,
        # when Wrot < 0, this is the same as rotate it 180ª
        if Wrot >= 0.0:
            circleTrayectory = 90.0 - np.rad2deg(footAngle - self.alpha)
        else:
            circleTrayectory = 270.0 - np.rad2deg(footAngle - self.alpha)

        stepOffset = 0.75
        if phi <= stepOffset:  # stance phase
            phiStance = phi / stepOffset
            stepX_long, stepY_long, stepZ_long = self.calculateStance(
                phiStance, V, angle
            )  # longitudinal step
            stepX_rot, stepY_rot, stepZ_rot = self.calculateStance(
                phiStance, Wrot, circleTrayectory
            )  # rotational step
        #            print(phi,phiStance, stepX_long)
        else:  # swing phase
            phiSwing = (phi - stepOffset) / (1 - stepOffset)
            stepX_long, stepY_long, stepZ_long = self.calculateBezier_swing(
                phiSwing, V, angle
            )  # longitudinal step
            stepX_rot, stepY_rot, stepZ_rot = self.calculateBezier_swing(
                phiSwing, Wrot, circleTrayectory
            )  # rotational step

        if centerToFoot[1] > 0:  # define the sign for every quadrant
            if stepX_rot < 0:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot ** 2 + stepY_rot ** 2), r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot ** 2 + stepY_rot ** 2), r)
        else:
            if stepX_rot < 0:
                self.alpha = np.arctan2(np.sqrt(stepX_rot ** 2 + stepY_rot ** 2), r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot ** 2 + stepY_rot ** 2), r)

        coord = np.empty(3)
        coord[0] = stepX_long + stepX_rot
        coord[1] = stepY_long + stepY_rot
        coord[2] = stepZ_long + stepZ_rot

        return coord

    def loop(self, V, angle, Wrot, dT, offset: np.ndarray, b2f: np.matrix) -> np.matrix:
        """computes step trajectory for every foot

        b2f: local bodytoFeet (4x3 np.matrix)

        Defining length of the step and direction (0º -> forward; 180º -> backward), L which is like velocity command, its angle, period of time of each step, offset between each foot, and the initial vector from center of robot to feet.

        Returns:
            4x3 np.matrix (np.zeros([4, 3]))
        """
        if dT <= 0.01:
            dT = 0.01

        if self.phi >= 0.99:
            self.lastTime = time.time()
        self.phi = (time.time() - self.lastTime) / dT

        # now it calculates step trajectory for every foot
        step_coord = self.stepTrajectory(
            self.phi + offset[0],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(b2f[0, :])),
        )  # FR
        self.bodytoFeet[0, 0] = b2f[0, 0] + step_coord[0]
        self.bodytoFeet[0, 1] = b2f[0, 1] + step_coord[1]
        self.bodytoFeet[0, 2] = b2f[0, 2] + step_coord[2]

        step_coord = self.stepTrajectory(
            self.phi + offset[1],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(b2f[1, :])),
        )  # FL
        self.bodytoFeet[1, 0] = b2f[1, 0] + step_coord[0]
        self.bodytoFeet[1, 1] = b2f[1, 1] + step_coord[1]
        self.bodytoFeet[1, 2] = b2f[1, 2] + step_coord[2]

        step_coord = self.stepTrajectory(
            self.phi + offset[2],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(b2f[2, :])),
        )  # BR
        self.bodytoFeet[2, 0] = b2f[2, 0] + step_coord[0]
        self.bodytoFeet[2, 1] = b2f[2, 1] + step_coord[1]
        self.bodytoFeet[2, 2] = b2f[2, 2] + step_coord[2]

        step_coord = self.stepTrajectory(
            self.phi + offset[3],
            V,
            angle,
            Wrot,
            np.squeeze(np.asarray(b2f[3, :])),
        )  # BL
        self.bodytoFeet[3, 0] = b2f[3, 0] + step_coord[0]
        self.bodytoFeet[3, 1] = b2f[3, 1] + step_coord[1]
        self.bodytoFeet[3, 2] = b2f[3, 2] + step_coord[2]
        #

        return self.bodytoFeet
