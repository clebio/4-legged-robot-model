#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  4 14:51:58 2020

@author: miguel-asd

Updates on September, 2020
@author: clebio
"""

import numpy as np
import time
import logging
from sys import exit
from os import environ

from src.joystick import Joystick, setup_joystick
from src.arduino import get_ACM
from src.model import servo

JOYSTICK = False if environ.get("NO_JOY") else True

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

if JOYSTICK:
    joystick = setup_joystick()

ARDUINO = False if environ.get("NO_ARDUINO") else True
if ARDUINO:
    arduino = get_ACM()
else:
    logger.info("Skipping Arduino setup!")

lastTime = 0.0
interval = 0.03

arduino.clear()
# logger.info("Enabling relay")
# arduino.send("<RELAY#on>")


while True:
    now = time.time()
    if now - lastTime < interval:
        continue
    lastTime = now
    logger.debug(f"Loop at {lastTime} ms")

    if JOYSTICK:

        oh_joy = joystick.read()
        if isinstance(oh_joy[0], str) and oh_joy[0] == "Shutdown!":
            logger.info("Shutting down")
            if ARDUINO:
                arduino.send("<QUIT>")
                arduino.send("<RELAY#off>")
                line = arduino.arduino.readline().decode("utf-8").rstrip()
                logger.info(f"Shutdown response: {line}")
            logger.info("Done with calibration")
            exit(0)

    if ARDUINO:
        response = arduino.receive()
        logger.debug(f"Arduino sent: {response}")
        if len(response) == 6:
            arduinoLoopTime, battery, Xacc, Yacc, realRoll, realPitch = response

    angles = [90 for _ in range(12)]

    angles = servo.limit(angles)
    message = "<SERVO#{}>\n".format(
        "#".join(f"{i}~{round(c)}" for i, c in enumerate(angles))
    )

    log_message = " ".join(str(round(c, 2)) for c in angles)
    logger.info(message.strip())
    if ARDUINO:
        arduino.send(message)
