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

logging.basicConfig(level=logging.INFO)

loopTime = 0.0
interval = 0.2
freqMulti = 2
speed = 10

HEADERS = [
    "IMU",
    "SERVO",
    "LCD",
]

MOVING = False

rad2pulse = lambda x: int(-speed * np.rad2deg(-x)) + 1500


def parse(record):
    try:
        lines = record.decode("utf-8").strip()
        if not lines:
            return False
    except:
        lines = record
    try:
        lines = [l.strip() for l in lines.split("\n")]
    except:
        lines = [lines]
    for line in lines:
        try:
            header, message = line.split("#")
        except:
            header = ""
            message = line
        logging.info(f"Received [{header}]: {message}")
    return lines


if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    ser.flush()
    relay = False

    while True:
        if time.time() - loopTime < interval:
            continue
        loopTime = time.time()

        # TODO: https://github.com/pyserial/pyserial/blob/master/serial/tools/miniterm.py#L499
        record = ser.read(ser.in_waiting or 1)
        logging.info(record.decode("utf-8").strip())
        # parse(record)

        moves = []
        rad = np.sin(freqMulti * loopTime)
        for i in range(12):
            if MOVING:
                dist = rad2pulse(rad)
            else:
                dist = 90 + np.rad2deg(rad)
            moves.append(f"{i}~{dist}")
        message = "<SERVO#{}>\n".format("#".join(moves))

        state = "on" if relay else "off"
        logging.info(f"Toggling relay {state}")
        message = f"<RELAY#{state}>\n"
        relay = not relay

        # logging.info(f"Sending: {message}")
        ser.write(bytes(message, "utf-8"))
