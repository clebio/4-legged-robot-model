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
import logging
import glob

logging.basicConfig(level=logging.INFO)


def get_ACM(tty="/dev/ttyACM*"):
    """Connect to ACM device and return Arduino object

    # ls -l /dev | grep ACM to identify serial port of the arduino
    """
    acms = glob.glob(tty)
    try:
        found = ArduinoSerial(acms.pop(), timeout=1)
    except SerialException as e:
        logger.error(f"Failed to connect to Arduino: {e}")
        raise e
    return found


class ArduinoSerial:
    """Communicate with Arduino through serial
    
    @miguel-asd: clase para la comunicacion con arduino mediante serial, se inicia diciendo puerto y velocidad y en la funcion crea un string con el formato para la lectura. en la primera ejecucion hay que abrir el puerto.
    """

    def __init__(self, port, timeout=1, interval=1):
        self.arduino = serial.Serial(port, 115200, timeout=timeout)
        self.arduino.flush()
        self.arduino.setDTR(True)

        self.lastTime = 0.0
        self.previousMillis = 0.0
        self.interval = interval  # arduino loop running at 20 ms

    def serialSend(self, command):
        self.arduino.write(bytes(command, encoding="utf8"))
        self.lastTime = time.time()

    def test(self):
        x = self.arduino.read()
        logging.info(ord(x))

    def serialReceive(self):
        try:
            line = self.arduino.readline()
        except ValueError as e:
            print(f"Failed to read: {e}")
            results = False

        deline = line.decode("utf-8").rstrip()
        if not deline.startswith("<") and not deline.endswith(">"):
            logging.warning(f"Didn't find control markers: {deline}")
            return [0 for _ in range(6)]

        if not deline.startswith("DATA"):
            logging.warning(f"Didn't receive a data record: {deline}")
            return [0 for _ in range(6)]

        deline = deline.strip("<>").split("#")
        logging.info(deline)
        results = numpy.array(deline)

        self.arduino.flush()
        return results  # loopTime, battery, Xacc, Yacc, roll, pitch

    def close(self):
        self.arduino.close()
