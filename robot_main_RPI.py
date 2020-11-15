#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 20:31:07 2020

@author: miguel-asd
"""

import numpy as np
import time
import csv
import logging
from os import environ

from src.model.kinematics import Quadruped
from src.joystick import Joystick, setup_joystick
from src.arduino import ArduinoSerial, get_ACM
from src.model import servo
from src.model.gaitPlanner import trotGait
from src.model.stabilization import stabilize

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


"initial foot position"
# foot separation (0.182 -> tetta=0) and distance to floor
# Ydist = 0.18
# Xdist = 0.25
robot = Quadruped()
JOYSTICK = False if environ.get("NO_JOY") else True
if JOYSTICK:
    joystick = setup_joystick()

ARDUINO = False if environ.get("NO_ARDUINO") else True
if ARDUINO:
    arduino = get_ACM()
else:
    logging.info("Skipping Arduino setup!")
trot = trotGait()
control = stabilize()

# period of time (in seconds) of every step
dT = 0.4
interval = 0.03
loopTime = time.time()

# Save telemetry data.
LOG_TELEMETRY = environ.get("LOG_TELEMETRY", False)
FIELDNAMES = ["t", "roll", "pitch", "battery"]
if LOG_TELEMETRY:
    csv_file = open("telemetry/data.csv", "w")
    csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    csv_writer.writeheader()

while True:
    now = time.time()
    if now - loopTime < interval:
        continue
    loopTime = now

    commandPose, commandOrn, Vel, angle, Wrot, dT, compliantMode = (
        joystick.read() if JOYSTICK else 0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    if commandPose == "Shutdown!":
        logger.info("Shutting down")
        if ARDUINO:
            arduino.send("<QUIT>")
            # line = arduino.arduino.readline().decode("utf-8").rstrip()
            # logger.info(f"Shutdown response: {line}")
        break

    arduino_response = [0 for _ in range(6)]
    if ARDUINO:
        arduino_response = arduino.receive()
        arduinoLoopTime, battery, Xacc, Yacc, realRoll, realPitch = arduino_response
    else:
        arduinoLoopTime = loopTime
        battery = 0
        Xacc = 0
        Yacc = 0
        realRoll = 0
        realPitch = 0

    forceAngle, Vcompliant = control.bodyCompliant(Xacc, Yacc, compliantMode)
    bodytoFeet = trot.loop(Vel + Vcompliant, angle + forceAngle, Wrot, dT)

    logger.debug(f"Solving for {commandOrn}, {commandPose}, {bodytoFeet}")
    pose = robot.solve(commandPose, commandOrn, bodytoFeet)

    angles = [np.rad2deg(p) for p in pose.flatten()]
    logger.debug(angles)
    # pulses = servo.convert(pulses)

    limited_angles = servo.limit(angles)
    message = "<SERVO#{}>\n".format(
        "#".join(f"{i}~{str(c)}" for i, c in enumerate(limited_angles))
    )

    log_message = " ".join("{: 3.2f}".format(round(c, 2)) for c in limited_angles)
    logger.info(f"Commands: {log_message}")
    if ARDUINO:
        arduino.send(message)

    # Upid_x , Upid_y , errorX , errorY , Upid_xorn , Upid_yorn = control.centerPoint(realPitch , realRoll)
    # orn[0] = Upid_xorn
    # orn[1] = Upid_yorn
    # pos[0] = Upid_x
    # pos[1] = Upid_y
    logger.debug(f"{loopTime}, {arduinoLoopTime}, {realRoll}, {realPitch}")

    if LOG_TELEMETRY:
        csv_writer.writerow(
            dict(zip(FIELDNAMES, [loopTime, realRoll, realPitch, battery]))
        )

if LOG_TELEMETRY:
    csv_file.close()
