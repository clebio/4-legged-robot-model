#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Updates on September, 2020
@author: clebio
"""

import serial
import time
from random import randint, random
import logging
import numpy as np

logging.basicConfig(level=logging.WARNING)

loopTime = 0.0
interval = 0.01
freqMulti = 2
speed = 10

HEADERS = [
    "IMU",
    "SERVO",
    "LCD",
]

rad2pulse = lambda x: int(-speed * np.rad2deg(-x)) + 1500

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    ser.flush()

    while True:
        if time.time() - loopTime >= interval:
            loopTime = time.time()

            moves = []
            rad = np.sin(freqMulti * loopTime)
            for i in range(12):
                # dist = rad2pulse(rad)
                dist = 90 + np.rad2deg(rad)
                moves.append(f"{i}~{dist}")
            message = "<SERVO#{}>\n".format("#".join(moves))
            logging.info(f"Sending: {message}")
        else:
            message = "Hello from Raspberry Pi!\n"

        ser.write(bytes(message, "utf-8"))

        line = ser.readline().decode("utf-8").rstrip()
        if line:
            logging.info(line)
