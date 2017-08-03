#!/usr/bin/env python
"""
FLOWERGIRL
"""

import asyncio
import enum
import errno
import functools
import json
import logging
import signal
import socket
import sys
import threading
import time
from math import pi

import flower_log
from motor import MotorSerial, Motor
from leg import Leg
from cannon import Cannon

# Left and right legs, #1 through 3 front to rear
class LegIndex(enum.Enum):
    L1 = 0
    L2 = 1
    L3 = 2
    R1 = 3
    R2 = 4
    R3 = 5

class ControlState(enum.Enum):
    ESTOP = 0
    STANDBY = 1
    INIT = 2
    SIT = 3
    STAND = 4
    WALK = 5
    YAW = 6

class Flowergirl(object):
    CMD_DEADBAND = 0.05   # Consider command values below this magnitude as zero
    GND_ARC = pi/3   # Max arc length
    GND_VEL = 0.5   # Max velocity on ground
    LIFT_VEL = 2   # Max lift velocity

    def __init__(self, loop,
                       leg_left_forward,
                       leg_left_center,
                       leg_left_rear,
                       leg_right_forward,
                       leg_right_center,
                       leg_right_rear,
                       cannon,
                       debug=False):
        self._loop = loop   # Event loop
        self.cmd_fwd = 0.0   # [-1, 1]
        self.cmd_yaw = 0.0   # [-1, 1]
        self.cmd_cannon  = False
        self.cmd_trigger = False   # Used to toggle between sit/stand
        self.cmd_estop  = True
        self.state = ControlState.ESTOP
        self.step = 0   # Switch between "left" and "right" steps
        self.stopflag = False
        self.watchdog_time = 0
        self.debug = debug

        self._state_change = True   # State change flag

        # Comm socket
        self.sock = None

        self.legs = {LegIndex.L1: leg_left_forward,
                     LegIndex.L2: leg_left_center,
                     LegIndex.L3: leg_left_rear,
                     LegIndex.R1: leg_right_forward,
                     LegIndex.R2: leg_right_center,
                     LegIndex.R3: leg_right_rear}
        self.cannon = cannon

        # Logging
        self._log = logging.getLogger("Flowergirl")
        self._log.setLevel(logging.DEBUG)
        self._log.addHandler(flower_log.ch)
        self._log.addHandler(flower_log.fh)

        self._control_task = self._loop.create_task(self.run_control())
        self._comm_task = self._loop.run_until_complete(asyncio.start_server(self.handle_recv, "", 55000, loop=self._loop))
        self._log.info("Waiting for command connection on {}".format(self._comm_task.sockets[0].getsockname()))

    @property
    def control_task(self):
        return self._control_task

    @property
    def comm_task(self):
        return self._comm_task

    def stop(self):
        for leg in self.legs.values():
            leg.stop()

        self._log.info("Stopping")
        self.stopflag = True

    def print_cmds(self):
        self._log.debug("Forward: {:+1.2f}   Yaw: {:+1.2f}   Cannon: {}   Estop: {}".format(self.cmd_fwd, self.cmd_yaw, "On" if self.cmd_cannon else "Off", self.cmd_estop))

    # All desired motor values should be set together by the controller
    def set_leg_velocities(self, l1, l2, l3, r1, r2, r3):
        self.legs[LegIndex.L1].set_vel_sp(l1)
        self.legs[LegIndex.L2].set_vel_sp(l2)
        self.legs[LegIndex.L3].set_vel_sp(l3)
        self.legs[LegIndex.R1].set_vel_sp(r1)
        self.legs[LegIndex.R2].set_vel_sp(r2)
        self.legs[LegIndex.R3].set_vel_sp(r3)

    def set_leg_positions(self, l1, l2, l3, r1, r2, r3):
        self.legs[LegIndex.L1].set_pos(l1)
        self.legs[LegIndex.L2].set_pos(l2)
        self.legs[LegIndex.L3].set_pos(l3)
        self.legs[LegIndex.R1].set_pos(r1)
        self.legs[LegIndex.R2].set_pos(r2)
        self.legs[LegIndex.R3].set_pos(r3)

    def set_state(self, state):
        if self.state == state:
            return
        self._log.info("State: {} -> {}".format(self.state, state))
        self.state = state
        self._state_change = True

    async def state_estop(self):
        if self._state_change:
            self.legs[LegIndex.L1].disable()
            self.legs[LegIndex.L2].disable()
            self.legs[LegIndex.L3].disable()
            self.legs[LegIndex.R1].disable()
            self.legs[LegIndex.R2].disable()
            self.legs[LegIndex.R3].disable()
            self.cannon.disable()
            self._state_change = False

        # If cannon button pressed, in E-stop, clear calibration values
        for leg in self.legs.values():
            if self.cmd_cannon and leg.calibrated:
                leg.clear_zero()

        if not self.cmd_estop:
            self.set_state(ControlState.STANDBY)

    async def state_standby(self):
        if self._state_change:
            self.legs[LegIndex.L1].enable()
            self.legs[LegIndex.L2].enable()
            self.legs[LegIndex.L3].enable()
            self.legs[LegIndex.R1].enable()
            self.legs[LegIndex.R2].enable()
            self.legs[LegIndex.R3].enable()
            self.cannon.enable()
            self._state_change = False
        if self.cmd_cannon:
            self.set_state(ControlState.INIT)

    async def state_init(self):
        """Zero the legs by turning them backwards until the toes hit the ground. Open-loop."""
        # Watchdog and E-stop don't work in this state due to the simplistic sleeps.
        if any([not leg.calibrated for leg in self.legs.values()]):
            self.set_leg_velocities(0, 0, -1, 0, 0, -1)
            self._log.info("Zeroing rear legs")
            await asyncio.sleep(1)
            self.set_leg_velocities(0, -1, -1, 0, -1, -1)
            self._log.info("Zeroing center legs")
            await asyncio.sleep(1)
            self.set_leg_velocities(-1, -1, -1, -1, -1, -1)
            self._log.info("Zeroing front legs")
            await asyncio.sleep(1)
            self.set_leg_velocities(-1, -1, 0, -1, -1, 0)
            await asyncio.sleep(1)
            self.set_leg_velocities(-1, 0, 0, -1, 0, 0)
            await asyncio.sleep(1)
            self.set_leg_velocities(0, 0, 0, 0, 0, 0)

            for leg in self.legs.values():
                leg.set_zero()
        else:
            self._log.info("Legs already calibrated")

        self.set_state(ControlState.SIT)

    async def state_sit(self):
        for leg in self.legs.values():
            leg.move_to(3*pi/4, 0.5)

        if self.cmd_trigger and all([leg.on_setpoint for leg in self.legs.values()]):
            if abs(self.cmd_fwd) > self.CMD_DEADBAND or abs(self.cmd_yaw) > self.CMD_DEADBAND:
                self._log.warn("Refusing to stand while joystick command is nonzero")
                return
            self.set_state(ControlState.STAND)

    async def state_stand(self):
        for leg in self.legs.values():
            leg.move_to(3*pi/2, 0.5)

        if self.cmd_trigger and all([leg.on_setpoint for leg in self.legs.values()]):
            self.set_state(ControlState.SIT)
        elif abs(self.cmd_fwd) > self.CMD_DEADBAND:
            self.set_state(ControlState.WALK)
        elif abs(self.cmd_yaw) > self.CMD_DEADBAND:
            self.set_state(ControlState.YAW)

    async def state_walk(self):
        # Scale step arc length (i.e., between foot touchdown and liftoff points) by forward command
        scale_left = min(max(self.cmd_fwd - self.cmd_yaw, 1), -1)
        scale_right = min(max(self.cmd_fwd + self.cmd_yaw, 1), -1)

        # If we get too close to negative scaling, we should switch to yaw mode.
        if scale_left < self.CMD_DEADBAND or scale_right < self.CMD_DEADBAND:
            self.set_state(ControlState.STAND)
            return

        if self.step is 0:
            self.legs[LegIndex.L1].move_to(3*pi/2 + self.GND_ARC/2 * scale_left,  self.GND_VEL  * scale_left)
            self.legs[LegIndex.L2].move_to(3*pi/2 - self.GND_ARC/2 * scale_left,  self.LIFT_VEL * scale_left)
            self.legs[LegIndex.L3].move_to(3*pi/2 + self.GND_ARC/2 * scale_left,  self.GND_VEL  * scale_left)
            self.legs[LegIndex.R1].move_to(3*pi/2 - self.GND_ARC/2 * scale_right, self.LIFT_VEL * scale_right)
            self.legs[LegIndex.R2].move_to(3*pi/2 + self.GND_ARC/2 * scale_right, self.GND_VEL  * scale_right)
            self.legs[LegIndex.R3].move_to(3*pi/2 - self.GND_ARC/2 * scale_right, self.LIFT_VEL * scale_right)
        else:
            self.legs[LegIndex.L1].move_to(3*pi/2 - self.GND_ARC/2 * scale_left,  self.LIFT_VEL * scale_left)
            self.legs[LegIndex.L2].move_to(3*pi/2 + self.GND_ARC/2 * scale_left,  self.GND_VEL  * scale_left)
            self.legs[LegIndex.L3].move_to(3*pi/2 - self.GND_ARC/2 * scale_left,  self.LIFT_VEL * scale_left)
            self.legs[LegIndex.R1].move_to(3*pi/2 + self.GND_ARC/2 * scale_right, self.GND_VEL  * scale_right)
            self.legs[LegIndex.R2].move_to(3*pi/2 - self.GND_ARC/2 * scale_right, self.LIFT_VEL * scale_right)
            self.legs[LegIndex.R3].move_to(3*pi/2 + self.GND_ARC/2 * scale_right, self.GND_VEL  * scale_right)

        if all([leg.on_setpoint for leg in self.legs.values()]):
            self.step = 1 - self.step   # Switch steps

        if abs(self.cmd_fwd) < self.CMD_DEADBAND:
            self.set_state(ControlState.STAND)

    async def state_yaw(self):
        # Scale step arc length (i.e., between foot touchdown and liftoff points) by forward command
        scale = self.cmd_yaw

        if self.step is 0:
            self.legs[LegIndex.L1].move_to(3*pi/2 - self.GND_ARC/2 * scale, -self.GND_VEL  * scale)
            self.legs[LegIndex.L2].move_to(3*pi/2 + self.GND_ARC/2 * scale, -self.LIFT_VEL * scale)
            self.legs[LegIndex.L3].move_to(3*pi/2 - self.GND_ARC/2 * scale, -self.GND_VEL  * scale)
            self.legs[LegIndex.R1].move_to(3*pi/2 - self.GND_ARC/2 * scale,  self.LIFT_VEL * scale)
            self.legs[LegIndex.R2].move_to(3*pi/2 + self.GND_ARC/2 * scale,  self.GND_VEL  * scale)
            self.legs[LegIndex.R3].move_to(3*pi/2 - self.GND_ARC/2 * scale,  self.LIFT_VEL * scale)
        else:
            self.legs[LegIndex.L1].move_to(3*pi/2 + self.GND_ARC/2 * scale, -self.LIFT_VEL * scale)
            self.legs[LegIndex.L2].move_to(3*pi/2 - self.GND_ARC/2 * scale, -self.GND_VEL  * scale)
            self.legs[LegIndex.L3].move_to(3*pi/2 + self.GND_ARC/2 * scale, -self.LIFT_VEL * scale)
            self.legs[LegIndex.R1].move_to(3*pi/2 + self.GND_ARC/2 * scale,  self.GND_VEL  * scale)
            self.legs[LegIndex.R2].move_to(3*pi/2 - self.GND_ARC/2 * scale,  self.LIFT_VEL * scale)
            self.legs[LegIndex.R3].move_to(3*pi/2 + self.GND_ARC/2 * scale,  self.GND_VEL  * scale)

        if all([leg.on_setpoint for leg in self.legs.values()]):
            self.step = 1 - self.step   # Switch steps

        if abs(self.cmd_fwd) > self.CMD_DEADBAND or abs(self.cmd_yaw) < self.CMD_DEADBAND:
            self.set_state(ControlState.STAND)

    def pet_watchdog(self):
        self.watchdog_time = time.time()

    @property
    def watchdog_alive(self):
        return not time.time() - self.watchdog_time > 1

    def handle_cannon(self, cmd):
        """Handle cannon command"""
        if self.state in [ControlState.ESTOP, ControlState.STANDBY, ControlState.INIT]:
            pass
        elif cmd and not self.cmd_cannon:
            self.cannon.on()
        elif not cmd and self.cmd_cannon:
            self.cannon.off()

        self.cmd_cannon = cmd

    async def run_control(self):
        """High-level command state machine"""
        self.set_state(ControlState.ESTOP)

        state_funcs = {ControlState.ESTOP:   self.state_estop,
                       ControlState.STANDBY: self.state_standby,
                       ControlState.INIT:    self.state_init,
                       ControlState.SIT:     self.state_sit,
                       ControlState.STAND:   self.state_stand,
                       ControlState.WALK:    self.state_walk,
                       ControlState.YAW:     self.state_yaw}

        last_update = time.time()
        update_count = 0
        while not self.stopflag:
            now = time.time()
            update_count += 1
            if now - last_update > 1:
                self._log.debug("Command freq: {} Hz".format(update_count))
                last_update = now
                update_count = 0

            # Unconditionally set Estop if commanded
            if not self.watchdog_alive:
                self.cmd_estop = True
            if self.cmd_estop:
                self.set_state(ControlState.ESTOP)

            await state_funcs[self.state]()
            await asyncio.sleep(0.005)   # TODO(syoo): do this properly

    async def handle_recv(self, reader, writer):
        """Command connection callback"""
        peername = writer.get_extra_info('peername')
        self._log.info("Received connection from {}".format(peername))
        # TODO(syoo): Limit to one connection

        self.cmd_estop = True
        while not self.stopflag:
            r = await reader.read(1024)

            if len(r) == 0:
                self._log.info("Client {} closed connection".format(peername))
                break

            try:
                cmd = json.loads(r)

                # Sanity check
                if abs(cmd["fwd"]) > 1.0:
                    self._log.error("Forward velocity must be within [-1, 1]")
                elif abs(cmd["yaw"]) > 1.0:
                    self._log.error("Yaw velocity must be within [-1, 1]")
                elif type(cmd["cannon"]) is not int:
                    self._log.error("Cannon command must be a boolean")
                elif type(cmd["estop"]) is not bool:
                    self._log.error("Estop command must be a boolean")
                    self.cmd_estop = True   # Assume Estop if input is bad
                else:
                    self.cmd_fwd = cmd["fwd"]
                    self.cmd_yaw = cmd["yaw"]
                    self.handle_cannon(cmd["cannon"])
                    self.cmd_trigger = cmd["trigger"]
                    self.cmd_estop = cmd["estop"]

                self.pet_watchdog()

                #self.print_cmds()
            except json.JSONDecodeError as e:
                self._log.error("JSON decode error: {}".format(e))

        self.cmd_estop = True
        writer.close()

if __name__ == "__main__":
    print("FLOWER TIME <3")
    print("")
    with open("robot_banner.txt") as f:
        for l in f.readlines():
            sys.stdout.write(l)
    print("")

    loop = asyncio.get_event_loop()

    #mc1 = MotorSerial("/dev/m1", 230400, 1)
    #mc2 = MotorSerial("/dev/m2", 230400, 1)
    #mc3 = MotorSerial("/dev/m3", 230400, 1)
    mct = MotorSerial("/dev/ttyACM0", 230400, 1)   # DEBUG(syoo)

    l1 = Leg(loop, "L1", Motor(loop, mct, 0))
    l2 = Leg(loop, "L2", Motor(loop, mct, 0))
    l3 = Leg(loop, "L3", Motor(loop, mct, 0))
    r1 = Leg(loop, "R1", Motor(loop, mct, 1), True)
    r2 = Leg(loop, "R2", Motor(loop, mct, 1), True)
    r3 = Leg(loop, "R3", Motor(loop, mct, 1), True)
    cn = Cannon(loop, "Quiet", Motor(loop, mct, 0))

    f = Flowergirl(loop, l1, l2, l3, r1, r2, r3, cn)

    def sig_handler(signal, frame):
        f.stop()
    signal.signal(signal.SIGINT, sig_handler)

    try:
        print("Entering event loop")
        print("Ctrl+C to stop")
        loop.run_until_complete(f.control_task)
    finally:
        print("Closing event loop")
        loop.close()

    print("no more flowers :(")
