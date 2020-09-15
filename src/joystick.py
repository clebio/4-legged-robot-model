#!/usr/bin/env python
# -*- coding: utf-8 -*-

from evdev import InputDevice, categorize, ecodes, list_devices
import numpy as np
import logging
import selectors


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class Joystick:
    def __init__(self, path):
        self.L3 = np.array([0.0, 0.0])
        self.R3 = np.array([0.0, 0.0])

        self.x = 0
        self.triangle = 0
        self.circle = 0
        self.square = 0

        self.T = 0.4
        self.V = 0.0
        self.angle = 0.0
        self.Wrot = 0.0
        self.compliantMode = False
        self.poseMode = False
        self.CoM_pos = np.zeros(3)
        self.CoM_orn = np.zeros(3)
        self.calibration = 0
        self.offset = 127

        self.device = InputDevice(path)
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.device, selectors.EVENT_READ)

    def unregister(self):
        self.device.close()
        self.selector.unregister(self.device)
        self.selector.close()

    def read(self):
        oldVal = self.offset
        valIdx = 0
        for key, _ in self.selector.select():
            for event in key.fileobj.read():
                logger.debug(event)

                if event.code == ecodes.BTN_SELECT:
                    self.unregister()
                    return ["Shutdown!", *np.zeros(6)]
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        if event.code == 17:  # up arrow
                            self.CoM_pos[2] += 0.002
                        if event.code == 545:  # down arrow
                            self.CoM_pos[2] -= 0.002
                        if event.code == 547:  # right arrow
                            self.T += 0.05
                        if event.code == 546:  # left arrow
                            self.T -= 0.05
                        if event.code == 308:  # square

                            self.compliantMode = not self.compliantMode
                        if event.code == 307:  # triangle
                            self.poseMode = not self.poseMode

                        if event.code == 310:  # R1
                            self.calibration += 5
                        if event.code == 313:  # R2
                            self.calibration -= 0.0005

                        if event.code == 311:  # L1
                            self.calibration -= 5
                        if event.code == 312:  # L2
                            self.CoM_orn[0] += 0.0005
                    else:
                        logger.debug("Unassigned button")
                ########################################  for my own joystick
                #      ^           #     ^            #
                #    ABS_Y         #    ABS_RY        #
                #  ←─────→ ABS_X #  ←─────→ ABS_RX   #
                #     ↓           #     ↓            #
                #######################################
                if event.type == ecodes.EV_ABS:
                    absevent = categorize(event)
                    ty = absevent.event.type
                    co = absevent.event.code
                    val = absevent.event.value

                    # TODO:
                    # Sometimes the inputs from the joystick jump around erratically
                    # -32010 -32000 200 -32001 -32011 ...
                    # when this happens, we should ignore those inputs temporarily
                    if abs(oldVal - val) > self.offset and valIdx < 10:
                        logging.debug(f"Input event fluctuations! {val}|{oldVal}")
                        oldVal = val
                        valIdx += 1
                        continue
                    oldVal = val

                    if ecodes.bytype[ty][co] == "ABS_HAT0X":
                        self.L3[0] = val - self.offset
                    elif ecodes.bytype[ty][co] == "ABS_HAT0Y":
                        self.L3[1] = val - self.offset

                    elif ecodes.bytype[ty][co] == "ABS_RX":
                        self.R3[0] = val - self.offset
                    elif ecodes.bytype[ty][co] == "ABS_RY":
                        self.R3[1] = val - self.offset

        if self.poseMode == False:
            self.V = np.sqrt(self.L3[1] ** 2 + self.L3[0] ** 2) / 100.0
            self.angle = np.rad2deg(np.arctan2(-self.L3[0], -self.L3[1]))
            self.Wrot = -self.R3[0] / 250.0
            #        Lrot = 0.
            if self.V <= 0.035:
                self.V = 0.0
            if self.Wrot <= 0.035 and self.Wrot >= -0.035:
                self.Wrot = 0.0
        else:
            self.CoM_orn[0] = np.deg2rad(self.R3[0] / 3)
            self.CoM_orn[1] = np.deg2rad(self.L3[1] / 3)
            self.CoM_orn[2] = -np.deg2rad(self.L3[0] / 3)
            self.CoM_pos[0] = -self.R3[1] / 5000

        return (
            self.CoM_pos,
            self.CoM_orn,
            self.V,
            -self.angle,
            -self.Wrot,
            self.T,
            self.compliantMode,
        )


def setup_joystick():
    devices = list_devices()
    if not devices:
        raise ValueError("Failed to list any joysticks")
    for device in devices:
        try:
            logger.info(f"Trying to use {device} as joystick...")
            joystick = Joystick(device)
            # joystick = Joystick("/dev/input/event1")  # need to specify the event route
        except:
            raise IOError(f"Failed getting {device}")
    return joystick


if __name__ == "__main__":
    joystick = setup_joystick()
    while True:
        logger.info(joystick.read())
