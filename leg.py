#!/usr/bin/env python
"""
Flowergirl leg control
"""

from motor import Motor
import serial
import time

class Leg(object):
    def __init__(self, motor, reverse=False):
        self._motor = motor
        self._reverse = reverse   # Reverse rotation axis

        self._zero = None   # Leg zero position in radians

    @property
    def calibrated(self):
        return not (self._zero == None)

    def estop(self):
        self._motor.estop()

    def set_vel(self, vel):
        """Set velocity in radians/second"""
        self._motor.set_vel(vel)

    def set_pos(self, pos):
        """Set position in radians"""
        if not self._zeroed:
            raise RuntimeError("Motor uncalibrated")
        raw_pos = pos + self._zero
        raise NotImplementedError

    def set_zero(self):
        self._zero = self._motor.get_pos()

if __name__ == "__main__":
    m = Motor("/dev/m1", 230400, 1)
    l = Leg(m)
    l.init()
    l.set_vel(1)
