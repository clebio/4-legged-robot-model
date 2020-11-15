#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 15:49:25 2020

@author: miguel-asd

Updates on September, 2020
@author: clebio
"""

import serial
import time
import numpy
import glob
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def get_ACM(tty="/dev/ttyACM*"):
    """Connect to ACM device and return Arduino object

    # ls -l /dev | grep ACM to identify serial port of the arduino
    """
    acms = glob.glob(tty)
    if not acms:
        logging.error(f"No ACM devices in glob")
        return False
    try:
        found = ArduinoSerial(acms.pop(), timeout=1)
        logger.info(f"Found {found} ACM device")
    except serial.SerialException as e:
        logger.error(f"Failed to connect to Arduino: {e}")
        return False
    return found


class ArduinoSerial:
    """Communicate with Arduino through serial

    @miguel-asd: clase para la comunicacion con arduino mediante serial, se inicia diciendo puerto y velocidad y en la funcion crea un string con el formato para la lectura. en la primera ejecucion hay que abrir el puerto.
    """

    def __init__(self, port, timeout=20, interval=1):
        self.arduino = serial.Serial(port, 115200, timeout=timeout)
        self.arduino.flush()
        self.arduino.setDTR(True)
        self.interval = interval

    def send(self, command):
        self.arduino.write(bytes(command, encoding="utf8"))

    def test(self):
        x = self.arduino.read()
        logger.info(ord(x))

    def clear(self):
        if self.arduino.in_waiting > 0:
            logger.debug(self.receive())

    def receive(self):
        lines = []
        try:
            while self.arduino.inWaiting():
                record = self.arduino.readline()
                record = record.decode("utf-8")
                lines.extend(l.strip("\n\r") for l in record.split("\n"))

        except ValueError as e:
            print(f"Failed to read: {e}")
            results = False
            return []
        return lines

    def close(self):
        self.arduino.close()
