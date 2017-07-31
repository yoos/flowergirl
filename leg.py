#!/usr/bin/env python
"""
Flowergirl leg control
"""

from math import pi
from motor import Motor
import serial
import threading
import time

class Leg(object):
    def __init__(self, name, motor, reverse=False):
        self._name = name
        self._motor = motor
        self._reverse = reverse   # Reverse rotation axis

        self._zero = None   # Leg zero position in radians

        # Control setpoints for move commands
        self._vel_sp = None   # Leg velocity setpoint in radians/second
        self._pos_sp = None   # Leg position setpoint in radians
        self._hys_lo = None   # Hysteresis low threshold
        self._hys_hi = None   # Hysteresis high threshold
        self._on_sp = False   # On-setpoint flag

        # Control thread
        self._stopflag = False
        self._thread = threading.Thread(target=self.run_control)
        self._thread.start()
        self._thread.join()

    def stop(self):
        """Stop control thread"""
        print("Stopping leg {} control thread".format(self._name))
        self._stopflag = True

    @property
    def calibrated(self):
        return not (self._zero == None)

    @property
    def on_setpoint(self):
        return self._on_sp

    def estop(self):
        """E-stop. This does not clear the leg's zero angle"""
        self._motor.estop()

    def get_pos(self):
        """Get position in radians"""
        raw_pos = self._motor.get_pos()

    def set_vel(self, vel):
        """Set velocity in radians/second"""
        if self._reverse:
            vel = -vel
        self._motor.set_vel(vel)

    def set_pos(self, pos):
        """Set position in radians"""
        if self._reverse:
            pos = -pos   # TODO(syoo): Is this right?
        if not self._zeroed:
            raise RuntimeError("Motor uncalibrated")
        raw_pos = pos + self._zero
        raise NotImplementedError

    def set_zero(self):
        """Set leg zero angle"""
        self._zero = self._motor.get_pos()

    def sp_err_pos(self, cur, sp):
        """Return error as [0, 2pi) from current leg angle to setpoint"""
        err = sp - cur
        if err < 0:
            err += 2*pi
        return err

    def sp_err(self, cur, sp):
        """Return error as (-pi, pi) from current leg angle to setpoint"""
        err = sp - cur
        if err < -pi:
            err += 2*pi
        elif err > pi:
            err -= 2*pi
        return err

    # Set setpoints
    def move_to(self, pos, vel, hys_lo=pi/72, hys_hi=pi/36):
        """Move leg at `vel` rad/s to `pos` radians, with some hysteresis."""
        self._vel_sp = vel
        self._pos_sp = pos
        self._hyst_lo = hys_lo
        self._hyst_hi = hys_hi

    # Run position controller
    def run_control(self):
        """Run leg position controller"""
        while not self._stopflag:
            if not self._pos_sp:
                return
            if not self._zeroed:
                raise RuntimeError("Motor uncalibrated")

            err = sp_err(self.get_pos(), self._pos_sp)
            if self._on_sp and abs(err) > self._hys_hi:
                self._on_sp = False
                self.set_vel(self._vel_sp)
            elif not self._on_sp and abs(err) > self._hys_lo:
                direction = err/abs(err)
                self.set_vel(direction * abs(self._vel_sp))
            elif not self._on_sp and abs(err) <= self._hys_lo:
                self._on_sp = True
                self.set_vel(0)

            time.sleep(0.001)   # TODO(syoo): properly implement control freq

        self.estop()

if __name__ == "__main__":
    m = Motor("/dev/m1", 230400, 1)
    l = Leg(m)
    l.init()
    l.set_vel(1)
