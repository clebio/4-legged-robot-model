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


def convert(angles):
    """Transform angle measures to servo microseconds, accounting for specific servo configs"""
    multiplier = -10.822
    pulse = np.empty([len(servo_configs)])

    for k, s in enumerate(servo_configs):
        pulse[k] = int(s.rotation + multiplier * np.rad2deg(-angles[k])) + s.center
    return pulse
