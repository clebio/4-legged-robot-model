#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 18:30:37 2020

@author: miguel-asd
"""
import numpy as np
from simple_pid import PID


class stabilize:
    def __init__(self):
        self.CoM = np.array([-0.01, 0.0, 0.01])

        self.pidX = PID(-0.0005, 0.0, 0.00001, setpoint=0.0)
        self.pidY = PID(0.00025, 0.0, 0.00001, setpoint=0.0)
        self.pidRoll = PID(-0.005, 0.0, 0.0001, setpoint=0.0)
        self.pidPitch = PID(-0.005, 0.0, 0.0001, setpoint=0.0)

        self.pidX.sample_time = 0.025
        self.pidY.sample_time = 0.025
        self.pidRoll.sample_time = 0.025
        self.pidPitch.sample_time = 0.025

        self.pidX.output_limits = (-0.03, 0.03)
        self.pidY.output_limits = (-0.03, 0.03)
        self.pidRoll.output_limits = (-np.pi / 4, np.pi / 4)
        self.pidPitch.output_limits = (-np.pi / 4, np.pi / 4)

        self.collision = False
        self.forceAngle = 0.0
        self.Lci = 0.0
        self.Lcompliant = []
        self.i = 0

    def centerPoint(self, actualPitch, actualRoll):

        Upid_xorn = self.pidRoll(actualRoll)
        Upid_yorn = self.pidPitch(actualPitch)
        Upid_x = self.pidX(actualPitch)
        Upid_y = self.pidY(actualRoll)

        return Upid_x, Upid_y, Upid_xorn, Upid_yorn

    def bodyCompliant(self, Xacc, Yacc, compliant=True):
        if not compliant:
            self.forceAngle = 0.0
            self.Lci = 0.0
            self.i = 0
            return self.forceAngle, self.Lci

        if Xacc >= 7000 or Yacc >= 7000:
            self.collision = True
            self.forceAngle = np.rad2deg(np.arctan2(Yacc, Xacc))
            fmoduli = np.sqrt(Xacc ** 2 + Yacc ** 2)
            self.Lci = fmoduli / 1000
            if self.Lci >= 0.6:
                self.Lci = 0.6
            self.Lcompliant = np.linspace(self.Lci, 0.0, 100)

        fmoduli = np.sqrt(Xacc ** 2 + Yacc ** 2)

        if self.collision == True:
            self.Lci = self.Lcompliant[self.i]
            if self.Lci >= 0.6:
                self.Lci = 0.6

            self.i += 1
            if self.Lcompliant[self.i] <= 0.0:
                self.collision = False
                self.i = 0
                self.forceAngle = 0.0

        return self.forceAngle, self.Lci
