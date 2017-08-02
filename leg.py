#!/usr/bin/env python
"""
Flowergirl leg control
"""

import asyncio
import argparse
import asyncio
import logging
import serial
import signal
import threading
import time

import flower_log
from math import pi
from motor import Motor, MotorSerial

class Leg(object):
    def __init__(self, loop, name, motor, reverse=False):
        self._loop = loop
        self._name = name
        self._motor = motor
        self._reverse = reverse   # Reverse rotation axis
        self._stopflag = False   # Control loop stop flag. Once set, loop will terminate.
        self._enabled = False   # Leg control enabled. When disabled, leg is limp.

        self._zero = None   # Leg zero position in radians

        # Control setpoints for move commands
        self._vel_sp = None   # Leg velocity setpoint in radians/second
        self._pos_sp = None   # Leg position setpoint in radians
        self._hys_lo = None   # Hysteresis low threshold
        self._hys_hi = None   # Hysteresis high threshold
        self._on_sp = False   # On-setpoint flag

        # Logging
        self._log = logging.getLogger("Leg {}".format(self._name))
        self._log.setLevel(logging.INFO)
        self._log.addHandler(flower_log.handler)

        self._task = self._loop.create_task(self.run())

    @property
    def task(self):
        return self._task

    @property
    def calibrated(self):
        return not (self._zero == None)

    @property
    def on_setpoint(self):
        return self._on_sp

    @property
    def pos(self):
        """Get position in radians"""
        raw_pos = self._motor.pos
        p = raw_pos   # TODO(syoo)
        return p

    def stop(self):
        """Stop control thread"""
        self._log.info("Stopping")
        self._motor.stop()
        self._stopflag = True

    def disable(self):
        """Disable leg. This does not clear the leg's zero angle, but the leg should go limp."""
        self._motor.disable()
        self._vel_sp = None
        self._pos_sp = None
        self._enabled = False
        self._log.info("Disabled")

    def enable(self):
        """Enable leg"""
        self._motor.enable()
        self._enabled = True
        self._log.info("Enabled")

    def set_vel_sp(self, vel):
        """Set velocity in radians/second"""
        if self._reverse:
            vel = -vel
        self._motor.set_vel_sp(vel)

    def set_zero(self):
        """Set leg zero angle"""
        self._zero = self._motor.pos
        self._log.info("Zero angle: {}".format(self._zero))

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

    def move_to(self, pos, vel, hys_lo=pi/72, hys_hi=pi/36):
        """Move leg at `vel` rad/s to `pos` radians, with some hysteresis."""
        self._vel_sp = vel
        self._pos_sp = pos
        self._hys_lo = hys_lo
        self._hys_hi = hys_hi

    async def run(self):
        """Run leg position controller"""
        last_update = time.time()
        update_count = 0
        while not self._stopflag:
            now = time.time()
            update_count += 1
            if now - last_update > 1:
                self._log.debug("Control freq: {} Hz".format(update_count))
                last_update = now
                update_count = 0

            if not self._enabled or self._pos_sp is None:
                await asyncio.sleep(0.05)
                continue
            if not self.calibrated:
                raise RuntimeError("Motor uncalibrated")

            err = self.sp_err(self.pos, self._pos_sp)
            if self._on_sp and abs(err) > self._hys_hi:
                self._on_sp = False
                self._motor.set_vel_sp(self._vel_sp)
            elif not self._on_sp and abs(err) > self._hys_lo:
                direction = err/abs(err)
                self._motor.set_vel_sp(direction * abs(self._vel_sp))
            elif not self._on_sp and abs(err) <= self._hys_lo:
                self._on_sp = True
                self._motor.set_vel_sp(0)

            await asyncio.sleep(0.002)   # TODO(syoo): properly implement control freq

        # Disable once done running
        self.disable()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Flowergirl leg tester')
    parser.add_argument('--dev', type=str, help='Serial device', required=True)
    parser.add_argument('--baud', type=int, help='Baudrate', default=921600)
    parser.add_argument('--index', type=int, help='Motor index', default=0)

    args = parser.parse_args()

    # Motor control
    mser = MotorSerial(args.dev, args.baud, 5)
    loop = asyncio.get_event_loop()
    m = Motor(loop, mser, args.index)

    # Leg control
    l = Leg(loop, "Test", m)
    l.enable()
    l.set_zero()
    l.move_to(0, 1)

    # Process inputs
    async def proc_inputs():
        while not leg_task.done():
            pos,vel = 0,0
            try:
                i = await loop.run_in_executor(None, input, "\nInput desired leg position and velocity (comma-separated):\n\n")
                tokens = i.split(',')
                pos = float(tokens[0])
                if len(tokens) < 2:
                    print("Assuming vel = 0.5")
                    vel = 0.5
                else:
                    vel = float(tokens[1])
                print("Moving leg to pos {} rad at {} rad/s".format(pos, vel))
                l.move_to(pos, vel)
            except Exception as e:
                print("Invalid input")

    loop.create_task(proc_inputs())   # This still crashes and burns when the event loop stops..

    # Stop leg control loop on SIGINT
    signal.signal(signal.SIGINT, lambda s,f: l.stop())

    # Loop!
    loop.run_until_complete(l.task)
    loop.close()
