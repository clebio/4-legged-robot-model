#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 18:16:57 2020

@author: miguel-asd

Updates on September, 2020
@author: clebio
"""

import numpy as np


def init_robot():
    # foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
    Xdist = 0.20
    Ydist = 0.15
    height = 0.15
    B2F0 = np.array(
        # body frame to foot frame vector
        [
            [Xdist / 2, -Ydist / 2, -height],
            [Xdist / 2, Ydist / 2, -height],
            [-Xdist / 2, -Ydist / 2, -height],
            [-Xdist / 2, Ydist / 2, -height],
        ]
    )
    stance = np.array([0.5, 0.0, 0.0, 0.5])

    return B2F0, stance


def Rx(roll: float) -> np.matrix:
    """Rotation matrix arround x (roll)"""
    #    roll = np.radians(roll)
    return np.matrix(
        [
            [1, 0, 0, 0],
            [0, np.cos(roll), -np.sin(roll), 0],
            [0, np.sin(roll), np.cos(roll), 0],
            [0, 0, 0, 1],
        ]
    )


def Ry(pitch: float) -> np.matrix:
    """Rotation matrix arround y (pitch)"""
    #    pitch = np.radians(pitch)
    return np.matrix(
        [
            [np.cos(pitch), 0, np.sin(pitch), 0],
            [0, 1, 0, 0],
            [-np.sin(pitch), 0, np.cos(pitch), 0],
            [0, 0, 0, 1],
        ]
    )


def Rz(yaw: float) -> np.matrix:
    """Rotation matrix arround z (yaw)"""
    #    yaw = np.radians(yaw)
    return np.matrix(
        [
            [np.cos(yaw), -np.sin(yaw), 0, 0],
            [np.sin(yaw), np.cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )


def Rxyz(roll: float, pitch: float, yaw: float) -> np.matrix:
    if not all([roll, pitch, yaw]):
        return np.identity(4)

    return Rx(roll) * Ry(pitch) * Rz(yaw)


def RTmatrix(orientation: np.ndarray, position: np.ndarray) -> np.matrix:
    """compose translation and rotation"""
    roll, pitch, yaw = orientation
    x0, y0, z0 = position

    translation = np.matrix(
        [
            [1, 0, 0, x0],  # Translation matrix
            [0, 1, 0, y0],
            [0, 0, 1, z0],
            [0, 0, 0, 1],
        ]
    )
    rotation = Rxyz(roll, pitch, yaw)  # rotation matrix

    return rotation * translation


def transform(coord, rotation, translation):
    """transforms a vector to a desire rotation and translation"""
    vector = np.array([[coord[0]], [coord[1]], [coord[2]], [1]])

    tranformVector = RTmatrix(rotation, translation) * vector
    return np.array([tranformVector[0, 0], tranformVector[1, 0], tranformVector[2, 0]])
