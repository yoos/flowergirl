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
    def __init__(self, loop, name, motor, index, reverse=False, debug=False):
        self._loop = loop
        self._name = name
        self._motor = motor
        self._index = index   # Motor index
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
        self._log = logging.getLogger("{} leg".format(self._name))
        self._log.setLevel(logging.DEBUG)
        self._log.addHandler(flower_log.ch if not debug else flower_log.ch_dbg)
        self._log.addHandler(flower_log.fh)

        self._task = self._loop.create_task(self.run())

    @property
    def fake(self):
        return self._motor.fake

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
        if self.fake:
            return self._pos_sp
        pos = self._motor.pos[self._index] if not self._reverse else self.reverse_pos(self._motor.pos[self._index])
        if self._zero is not None:
            pos = (pos - self._zero) % (2*pi)
        return pos

    def stop(self):
        """Stop control thread"""
        self._stopflag = True
        self._log.info("Stop flag set")

    def disable(self):
        """Disable leg. This does not clear the leg's zero angle, but the leg should go limp."""
        self._motor.disable(self._index)
        self._vel_sp = None
        self._pos_sp = None
        self._enabled = False
        self._log.info("Disabled")

    def enable(self):
        """Enable leg"""
        self._motor.enable(self._index)
        self._enabled = True
        self._log.info("Enabled")

    def set_vel_sp(self, vel):
        """Set velocity in radians/second"""
        self._motor.set_vel_sp(self._index, vel)

    def set_zero(self):
        """Set leg zero angle"""
        self._zero = self._motor.pos[self._index] if not self._reverse else self.reverse_pos(self._motor.pos[self._index])
        self._vel_sp = 0
        self._pos_sp = None
        self._on_sp = True
        self._log.info("Zero angle: {:.2f}".format(self._zero))

    def clear_zero(self):
        """Clear leg zero angle"""
        self._zero = None
        self._vel_sp = 0
        self._pos_sp = None
        self._on_sp = False
        self._log.info("Zero angle cleared")

    def reverse_pos(self, pos):
        """Return the reverse position in radians. I.e., map [0, 2*pi) to [pi, 0]U(2*pi, pi)"""
        return (pi - pos) % (2*pi)

    def sp_err(self, cur, sp):
        """Return error as (-pi, pi) from current leg angle to setpoint, i.e., the closest angle from current position to setpoint"""
        err = sp - cur
        if err < -pi:
            err += 2*pi
        elif err > pi:
            err -= 2*pi
        return err

    def move_to(self, pos, vel, hys_lo=pi/36, hys_hi=pi/18):
        """Move leg at `vel` rad/s to `pos` radians, with some hysteresis. Position setpoints are wrapped to [0, 2pi)"""
        self._vel_sp = vel
        self._pos_sp = (pos % (2*pi)) if (pos is not None) else None
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
                self._log.debug("mp: {:.3f}  p: {:.2f}  z: {:.3f}  spp: {:.3f}  en: {}  freq: {} Hz".format(
                    self._motor.pos[self._index],
                    self.pos if self.pos is not None else -1,
                    self._zero if self._zero is not None else -1,
                    self._pos_sp if self._pos_sp is not None else -1,
                    self._enabled,
                    update_count))
                last_update = now
                update_count = 0

            # If disabled or we have no velocity setpoint, don't do anything.
            if not self._enabled or self._vel_sp is None:
                await asyncio.sleep(0.05)
                continue

            await asyncio.sleep(0.004)   # TODO(syoo): properly implement control freq

            # Recalculate setpoints for control (need to deal with reverse, etc)
            vel_sp = self._vel_sp if not self._reverse else -self._vel_sp
            pos_sp = self._pos_sp

            # If setting velocity only, do that.
            if pos_sp is None:
                self._motor.set_vel_sp(self._index, vel_sp)
                #self._log.info("Setting velocity: {}".format(vel_sp))
                continue

            # Try to reach the position setpoint with some hysteresis. I.e.,
            # try to move in the direction and velocity specified by `vel_sp`
            # towards the position specified by `pos_sp` until we are within
            # +/- `hys_lo` of the position setpoint. If the measured position
            # subsequently deviates beyond +/- `hys_lo` but stays within +/-
            # `hys_hi`, make corrective movements towards the position
            # setpoint, ignoring the direction implied by `vel_sp`. However, if
            # the position deviates past `hys_hi`, once again move in the
            # direction implied by `vel_sp` until we reach the position
            # setpoint.
            err = self.sp_err(self.pos, pos_sp)
            if self._on_sp:
                if abs(err) > self._hys_hi:
                    self._on_sp = False   # No longer on target
                elif abs(err) > self._hys_lo:
                    s = -1 if err < 0 else 1
                    self._motor.set_vel_sp(self._index, abs(vel_sp) * s)
                else:
                    self._motor.set_vel_sp(self._index, 0)

            if not self._on_sp:
                if abs(err) > self._hys_lo:
                    self._motor.set_vel_sp(self._index, vel_sp)   # No matter what, move in specified direction
                else:
                    self._on_sp = True   # On target
                    self._motor.set_vel_sp(self._index, 0)

        # Disable once done running
        self.disable()

        # Stop motor
        self._motor.stop()

        self._log.debug("EXIT")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Flowergirl leg tester')
    parser.add_argument('--dev', type=str, help='Serial device', required=True)
    parser.add_argument('--baud', type=int, help='Baudrate', default=230400)
    parser.add_argument('--index', type=int, help='Motor index', default=0)
    parser.add_argument('--reverse', help='Reverse rotation axis', action='store_true')
    args = parser.parse_args()

    # Motor control
    mser = MotorSerial(args.dev, args.baud, 1)
    loop = asyncio.get_event_loop()
    m = Motor(loop, mser, debug=True)

    # Leg control
    l = Leg(loop, "Test", m, args.index, reverse=args.reverse, debug=True)
    l.enable()

    # Process inputs
    async def proc_inputs():
        while not l.task.done():
            pos,vel = 0,0
            try:
                i = await loop.run_in_executor(None, input, "\nInput desired leg position and velocity (comma-separated):\n\n")
                if i == "z":
                    print("Zeroing leg")
                    l.set_zero()
                    continue
                tokens = i.split(',')
                pos = None if tokens[0] == "n" else float(tokens[0])
                if len(tokens) < 2:
                    print("Assuming vel = 0.5")
                    vel = 0.5
                else:
                    vel = float(tokens[1])
                print("Moving leg to pos {} rad at {} rad/s".format(pos, vel))
                l.move_to(pos, vel)
            except Exception as e:
                print("Invalid input")

    input_task = loop.create_task(proc_inputs())

    # Stop leg control loop on SIGINT
    signal.signal(signal.SIGINT, lambda s,f: l.stop())

    # Gather all tasks to wait on
    future = asyncio.gather(l.task, m.task, input_task)

    # Loop!
    loop.run_until_complete(future)
    loop.close()
