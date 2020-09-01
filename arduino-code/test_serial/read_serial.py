#!/usr/bin/env python3
import serial
import time
from random import randint

loopTime = 0.0
interval = 0.5

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    ser.flush()

    while True:
        if time.time() - loopTime >= randint(1, 3):
            loopTime = time.time()
            parts = [loopTime, 0, 3.1415, -1]
            message = "<{}>\n".format("#".join([str(p) for p in parts]))
            ser.write(bytes(message, "UTF-8"))
            continue
        else:
            ser.write(b"Hello from Raspberry Pi!\n")

        line = ser.readline().decode("utf-8").rstrip()
        print(line)
        time.sleep(interval)
