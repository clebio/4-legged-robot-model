#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 16:27:16 2020

@author: miguel-asd

Updates on September, 2020
@author: clebio
"""
import numpy as np
from collections import namedtuple


def limit(angles):
    """Limit servo angles to safe ranges

    Supports specific angle limits for a list of 12 servos
    """
    a = namedtuple("servo_limits", ["min", "max"])
    lims = {
        # Left Front
        0: a(60, 110),
        1: a(28, 110),
        2: a(60, 130),
        # Left Back
        3: a(42, 100),
        4: a(40, 180),
        5: a(35, 112),
        # Right front
        6: a(72, 120),
        7: a(34, 148),
        8: a(65, 120),
        # Right back
        9: a(58, 125),
        10: a(20, 165),
        11: a(55, 125),
    }
    if len(angles) > 12:
        raise KeyError("We only support 12 servos")

    for k, v in enumerate(angles):
        _min, _max = lims[k]
        if v < _min:
            v = _min
            angles[k] = v
        if v > _max:
            v = _max
            angles[k] = v
    return angles


def convert(angles):
    """Transform angle measures to servo microseconds, accounting for specific servo configs"""
    s = namedtuple("servo_config", ["center", "rotation"])

    NUM_SERVOS = 12
    servo_configs = [s(1000, 0) for _ in range(NUM_SERVOS)]
    #     s(950, 1),  # FR
    #     s(800, 1),
    #     s(1000, 90),
    #     s(1020, 1),  # FL
    #     s(570, 1),
    #     s(1150, 90),
    #     s(1060, 1),  # BR
    #     s(2335, 1),
    #     s(1200, 90),
    #     s(890, 1),  # BL
    #     s(710, 1),
    #     s(1050, 90),
    # ]

    multiplier = -10.822
    pulse = np.empty([len(servo_configs)])

    for k, s in enumerate(servo_configs):
        pulse[k] = int(s.rotation + multiplier * np.rad2deg(-angles[k])) + s.center
    return pulse
