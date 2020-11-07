#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Standing robot using IMU for stabilization
No joystick input, no walking

@author: Caleb Hyde
@date: September, 2020
"""

import numpy as np
import time
import csv
import logging
from os import environ

from src.model.kinematics import Quadruped
from src.model import servo
from src.model.stabilization import stabilize

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


"initial foot position"
# foot separation (0.182 -> tetta=0) and distance to floor
# Ydist = 0.18
# Xdist = 0.25
robot = Quadruped()

ARDUINO = False if environ.get("NO_ARDUINO") else True
if ARDUINO:
    from src.arduino import ArduinoSerial, get_ACM
    arduino = get_ACM()
else:
    logging.info("Skipping Arduino setup!")
control = stabilize()

# period of time (in seconds) of every step
dT = 0.4
interval = 0.030
loopTime = time.time()

# Save telemetry data.
LOG_TELEMETRY = environ.get("LOG_TELEMETRY", False)
FIELDNAMES = ["t", "roll", "pitch", "battery"]
if LOG_TELEMETRY:
    csv_file = open("telemetry/data.csv", "w")
    csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
    csv_writer.writeheader()

shutdown = False

# foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
Xdist = 0.20
Ydist = 0.15
height = 0.15
B2F0 = np.matrix(
    # body frame to foot frame vector
    [
        [Xdist / 2, -Ydist / 2, -height],
        [Xdist / 2, Ydist / 2, -height],
        [-Xdist / 2, -Ydist / 2, -height],
        [-Xdist / 2, Ydist / 2, -height],
    ]
)

rotation = 0.0
dT = 0.4
commandPose = np.zeros(3)
commandOrn = np.zeros(3)
compliantMode = True

while True:
    now = time.time()
    if now - loopTime < interval:
        continue
    loopTime = now

    if shutdown:
        logger.info("Shutting down")
        if ARDUINO:
            arduino.send("<QUIT>")
            # line = arduino.arduino.readline().decode("utf-8").rstrip()
            # logger.info(f"Shutdown response: {line}")
        break

    arduino_response = [0 for _ in range(6)]
    if ARDUINO:
        response = arduino.receive()
        if "BNO055" in response:
            try:
                header, message = response.strip("<>").split("#")
                x, y, z, ax, ay, az, aw = message.split(" ")
                logging.info(
                    f"Received {header}: {x}, {y}, {z}, {ax}, {ay}, {az}, {aw}"
                )
                Xacc, Yacc, realRoll, realPitch = x, y, ax, ay
            except:
                logging.warning(f"Failed to parse arduino: [{response}]")
                Xacc, Yacc, realRoll, realPitch = 0, 0, 0, 0
    else:
        arduinoLoopTime = loopTime
        battery = 0
        Xacc = 0
        Yacc = 0
        realRoll = 0
        realPitch = 0

    forceAngle, Vcompliant = control.bodyCompliant(Xacc, Yacc, compliantMode)

    logger.debug(f"Solving for {commandOrn}, {commandPose}, {B2F0}")
    pose = robot.solve(commandOrn, commandPose, B2F0)

    angles = [np.rad2deg(p) for p in pose.flatten()]
    logger.debug(angles)
    # pulses = servo.convert(pulses)

    angles = servo.limit(angles)
    message = "<SERVO#{}>\n".format(
        "#".join(f"{i}~{str(c)}" for i, c in enumerate(angles))
    )

    log_message = " ".join("{: 3.2f}".format(round(c, 2)) for c in angles)
    logger.info(f"Commands: {log_message}")
    if ARDUINO:
        arduino.send(message)

    if LOG_TELEMETRY:
        csv_writer.writerow(
            dict(zip(FIELDNAMES, [loopTime, realRoll, realPitch, battery]))
        )

if LOG_TELEMETRY:
    csv_file.close()
