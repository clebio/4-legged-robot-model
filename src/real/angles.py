#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Updates on September, 2020
@author: clebio
"""

import logging
import time
from src.arduino import get_ACM
from src.model import servo
import sys

lastTime = 0.0
interval = 0.03

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
arduino = get_ACM()
if not arduino:
    logging.error("No serial device found, are we running on the Raspberry Pi, andis the Arduino connected?")
    sys.exit(1)
arduino.clear()
arduino.send("<QUIT>")
logger.info("Enabling relay")
arduino.send("<RELAY#on>")

while True:
    now = time.time()
    if now - lastTime < interval:
        continue
    lastTime = now
    logger.debug(f"Loop at {lastTime} ms")
    logger.info("\n".join(arduino.receive())) 
    
    angles = [90 for _ in range(12)]

    s = input("Servo number to adjust ('q' to quit)? ")
    if s == "q":
        arduino.send("<QUIT>")
        arduino.send("<RELAY#off>")
        sys.exit()
    try:
        s = int(s)
    except:
        logging.error("Please input integers")

    angle = int(input("Angle? "))
    angles[s] = angle

    angles = servo.limit(angles)
    message = "<SERVO#{}>\n".format(
        "#".join(f"{i}~{round(c)}" for i, c in enumerate(angles))
    )

    log_message = " ".join(str(round(c, 2)) for c in angles)
    logger.info(f"Commands: {log_message}")
    arduino.send(message)
