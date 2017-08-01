#!/usr/bin/env python
"""
Flowergirl leg control
"""

from math import pi
from motor import Motor
import asyncio
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
        return
        self._stopflag = False
        self._thread = threading.Thread(target=self.run_control)
        print("Leg {} control thread starting".format(self._name))
        self._thread.start()
        self._thread.join()
        print("Leg {} control thread stopped".format(self._name))

    def stop(self):
        """Stop control thread"""
        print("Leg {} control thread stopping".format(self._name))
        self._stopflag = True

    @property
    def calibrated(self):
        return not (self._zero == None)

    @property
    def on_setpoint(self):
        return self._on_sp

    @asyncio.coroutine
    def estop(self):
        """E-stop. This does not clear the leg's zero angle"""
        self._motor.disable()

    @asyncio.coroutine
    def get_pos(self):
        """Get position in radians"""
        raw_pos = self._motor.get_pos()   # TODO(syoo)

    @asyncio.coroutine
    def set_vel(self, vel):
        """Set velocity in radians/second"""
        if self._reverse:
            vel = -vel
        self._motor.set_vel(vel)

    @asyncio.coroutine
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

    @asyncio.coroutine
    def move_to(self, pos, vel, hys_lo=pi/72, hys_hi=pi/36):
        """Move leg at `vel` rad/s to `pos` radians, with some hysteresis."""
        self._vel_sp = vel
        self._pos_sp = pos
        self._hyst_lo = hys_lo
        self._hyst_hi = hys_hi

    @asyncio.coroutine
    def run_control(self):
        """Run leg position controller"""
        last_est_time = time.time()
        loop_count = 0
        while not self._stopflag:
            now = time.time()
            loop_count += 1

            if not self._pos_sp:
                continue
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

            yield from asyncio.sleep(0.001)   # TODO(syoo): properly implement control freq

            if now - last_est_time > 1:
                print("[{:.3f}] Leg {} control freq: {} Hz".format(now, self._name, loop_count))
                last_est_time = now
                loop_count = 0

        self.estop()

if __name__ == "__main__":
    m = Motor("/dev/m1", 230400, 1)
    l = Leg(m)
    l.init()
    l.set_vel(1)
