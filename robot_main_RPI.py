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

from src.kinematic_model import robotKinematics
from src.joystick import Joystick, setup_joystick
from src.arduino import ArduinoSerial, get_ACM
from src import servo
from src.gaitPlanner import trotGait
from src.stabilization import stabilize

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


robotKinematics = robotKinematics()
joystick = setup_joystick()

ARDUINO = False if environ.get("NO_ARDUINO") else True
if ARDUINO:
    arduino = get_ACM()
else:
    logging.info("Skipping Arduino setup!")
trot = trotGait()
control = stabilize()

# robot properties
"""initial safe position"""
# angles
# targetAngs = np.array([0 , np.pi/4 , -np.pi/2, 0 ,#BR
#                         0 , np.pi/4 , -np.pi/2, 0 ,#BL
#                         0 , np.pi/4 , -np.pi/2, 0 ,#FL
#                         0 , np.pi/4 , -np.pi/2, 0 ])#FR

# FR_0  to FR_4
# FRcoord = np.matrix([0. , -3.6 , -0.15])
# FLcoord = np.matrix([0. ,  3.6 , -0.15])
# BRcoord = np.matrix([0. , -3.6 , -0.15])
# BLcoord = np.matrix([0. ,  3.6 , -0.15])


"initial foot position"
# foot separation (0.182 -> tetta=0) and distance to floor
# Ydist = 0.18
# Xdist = 0.25
height = 0.16
# body frame to foot frame vector (0.08/-0.11 , -0.07 , -height)
bodytoFeet0 = np.matrix(
    [
        [0.085, -0.075, -height],
        [0.085, 0.075, -height],
        [-0.11, -0.075, -height],
        [-0.11, 0.075, -height],
    ]
)

orn = np.array([0.0, 0.0, 0.0])
pos = np.array([0.0, 0.0, 0.0])

lastTime = time.time()
# t = []

# period of time (in seconds) of every step
dT = 0.4

# offset between each foot step in this order (FR,FL,BR,BL)
# [0. , 0.25 , 0.75 , 0.5] creep gait
offset = np.array([0.0, 0.5, 0.5, 0.0])

interval = 0.030

# Save telemetry data.
LOG_TELEMETRY = environ.get("LOG_TELEMETRY", True)
FIELDNAMES = ["t", "roll", "pitch", "battery"]
if LOG_TELEMETRY:
    csv_file = open("telemetry/data.csv", "w")
    csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    csv_writer.writeheader()

while True:
    now = time.time()
    loopTime = now - lastTime
    if now - lastTime < interval:
        continue
    logger.debug(now)
    lastTime = now

    commandPose, commandOrn, Vel, angle, Wrot, dT, compliantMode = joystick.read()
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

    # bodytoFeet = np.ones([4, 3])
    _, forceAngle, Vcompliant, _ = control.bodyCompliant(Xacc, Yacc, compliantMode)
    bodytoFeet = trot.loop(
        Vel + Vcompliant, angle + forceAngle, Wrot, dT, offset, bodytoFeet0
    )

    logger.debug(f"Solving for {orn}, {pos}, {bodytoFeet}")
    FRa, FLa, BRa, BLa, _ = robotKinematics.solve(
        orn + commandOrn, pos + commandPose, bodytoFeet
    )

    # The existing code takes (and returns) four lists of three numbers (quite reasonably, since the quadruped servos logically group that way). But for some of what we're doing, and in particular the Arduino serial comms, it's easier to just work with one flat list of 12. That's what this [*a, *b, *c. *d] syntax does (the asterisk is a "splat" operator, so this flattens the lists to one list). At some point (TODO), I'd like to standardize these interface contracts.
    angles = [np.rad2deg(p) for p in [*FRa, *FLa, *BRa, *BLa]]
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
            dict(zip(FIELDNAMES, [lastTime, realRoll, realPitch, battery]))
        )
csv_file.close()
