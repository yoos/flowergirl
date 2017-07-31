#!/usr/bin/env python
"""
FLOWERGIRL
"""

import enum
import errno
import json
import signal
import socket
import sys
import threading
import time
from math import pi

from motor import MotorSerial, Motor
from leg import Leg

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
    def __init__(self, leg_left_forward,
                       leg_left_center,
                       leg_left_rear,
                       leg_right_forward,
                       leg_right_center,
                       leg_right_rear,
                       debug=False):
        self.cmd_fwd = 0.0   # [-1, 1]
        self.cmd_yaw = 0.0   # [-1, 1]
        self.cmd_cannon  = False
        self.cmd_trigger = False   # Used to toggle between sit/stand
        self.cmd_estop  = True
        self.state = ControlState.ESTOP
        self.step = 0   # Switch between "left" and "right" steps
        self.stopflag = False
        self.debug = debug

        # Comm socket
        self.sock = None

        self.legs = {LegIndex.L1: leg_left_forward,
                     LegIndex.L2: leg_left_center,
                     LegIndex.L3: leg_left_rear,
                     LegIndex.R1: leg_right_forward,
                     LegIndex.R2: leg_right_center,
                     LegIndex.R3: leg_right_rear}

    def stop(self):
        for leg in self.legs.values():
            leg.stop()

        print("no more flowers :(")
        self.stopflag = True

    def print_cmds(self):
        print("Forward: {:+1.2f}   Yaw: {:+1.2f}   Cannon: {}   Estop: {}".format(self.cmd_fwd, self.cmd_yaw, "On" if self.cmd_cannon else "Off", self.cmd_estop))

    # All desired motor values should be set together by the controller
    def set_leg_velocities(self, l1, l2, l3, r1, r2, r3):
        self.legs[LegIndex.L1].set_vel(l1)
        self.legs[LegIndex.L2].set_vel(l2)
        self.legs[LegIndex.L3].set_vel(l3)
        self.legs[LegIndex.R1].set_vel(r1)
        self.legs[LegIndex.R2].set_vel(r2)
        self.legs[LegIndex.R3].set_vel(r3)

    def set_leg_positions(self, l1, l2, l3, r1, r2, r3):
        self.legs[LegIndex.L1].set_pos(l1)
        self.legs[LegIndex.L2].set_pos(l2)
        self.legs[LegIndex.L3].set_pos(l3)
        self.legs[LegIndex.R1].set_pos(r1)
        self.legs[LegIndex.R2].set_pos(r2)
        self.legs[LegIndex.R3].set_pos(r3)

    def set_state(self, state):
        print("[{:.3f}] State: {} -> {}".format(time.time(), self.state, state))
        self.state = state

    def state_estop(self):
        try:
            self.legs[LegIndex.L1].estop()
            self.legs[LegIndex.L2].estop()
            self.legs[LegIndex.L3].estop()
            self.legs[LegIndex.R1].estop()
            self.legs[LegIndex.R2].estop()
            self.legs[LegIndex.R3].estop()
        except NotImplementedError:
            pass

        self.cmd_fwd = 0.0
        self.cmd_yaw = 0.0
        self.cmd_cannon = 0

        if not self.cmd_estop:
            self.set_state(ControlState.STANDBY)

    def state_standby(self):
        if self.cmd_cannon:
            self.set_state(ControlState.INIT)

    def state_init(self):
        """Zero the legs by turning them backwards until the toes hit the ground. Open-loop."""
        try:
            self.set_leg_velocities(0, 0, -1, 0, 0, -1)
            time.sleep(1)
            self.set_leg_velocities(0, -1, -1, 0, -1, -1)
            time.sleep(1)
            self.set_leg_velocities(-1, -1, -1, -1, -1, -1)
            time.sleep(1)
            self.set_leg_velocities(-1, -1, 0, -1, -1, 0)
            time.sleep(1)
            self.set_leg_velocities(-1, 0, 0, -1, 0, 0)
            time.sleep(3)
            self.set_leg_velocities(0, 0, 0, 0, 0, 0)
        except NotImplementedError:
            pass
        for leg in self.legs.values():
            leg.set_zero()

        self.set_state(ControlState.SIT)

    def state_sit(self):
        for leg in self.legs.values():
            leg.move_to(pi/2, 1)

        if self.cmd_trigger and all([leg.on_setpoint for leg in self.legs.values()]):
            self.set_state(ControlState.STAND)

    def state_stand(self):
        for leg in self.legs.values():
            leg.move_to(3*pi/2, 0.5)

        if self.cmd_trigger and all([leg.on_setpoint for leg in self.legs.values()]):
            self.set_state(ControlState.SIT)
        elif abs(self.cmd_fwd) > 0.05:
            self.set_state(ControlState.WALK)
        elif abs(self.cmd_yaw) > 0.05:
            self.set_state(ControlState.YAW)

    def state_walk(self):
        # Scale step arc length (i.e., between foot touchdown and liftoff points) by forward command
        scale_left = min(max(self.cmd_fwd - self.cmd_yaw, 1), -1)
        scale_right = min(max(self.cmd_fwd + self.cmd_yaw, 1), -1)

        # If we get too close to negative scaling, we should switch to yaw mode.
        if scale_left < 0.05 or scale_right < 0.05:
            self.set_state(ControlState.STAND)
            return

        gnd_arc = pi/3   # Max arc length
        gnd_vel = 0.5   # Max velocity on ground
        lift_vel = 2   # Max lift velocity

        if self.step is 0:
            self.legs[LegIndex.L1].move_to(3*pi/2 + gnd_arc/2 * scale_left,  gnd_vel  * scale_left)
            self.legs[LegIndex.L2].move_to(3*pi/2 - gnd_arc/2 * scale_left,  lift_vel * scale_left)
            self.legs[LegIndex.L3].move_to(3*pi/2 + gnd_arc/2 * scale_left,  gnd_vel  * scale_left)
            self.legs[LegIndex.R1].move_to(3*pi/2 - gnd_arc/2 * scale_right, lift_vel * scale_right)
            self.legs[LegIndex.R2].move_to(3*pi/2 + gnd_arc/2 * scale_right, gnd_vel  * scale_right)
            self.legs[LegIndex.R3].move_to(3*pi/2 - gnd_arc/2 * scale_right, lift_vel * scale_right)
        else:
            self.legs[LegIndex.L1].move_to(3*pi/2 - gnd_arc/2 * scale_left,  lift_vel * scale_left)
            self.legs[LegIndex.L2].move_to(3*pi/2 + gnd_arc/2 * scale_left,  gnd_vel  * scale_left)
            self.legs[LegIndex.L3].move_to(3*pi/2 - gnd_arc/2 * scale_left,  lift_vel * scale_left)
            self.legs[LegIndex.R1].move_to(3*pi/2 + gnd_arc/2 * scale_right, gnd_vel  * scale_right)
            self.legs[LegIndex.R2].move_to(3*pi/2 - gnd_arc/2 * scale_right, lift_vel * scale_right)
            self.legs[LegIndex.R3].move_to(3*pi/2 + gnd_arc/2 * scale_right, gnd_vel  * scale_right)

        if all([leg.on_setpoint for leg in self.legs.values()]):
            self.step = 1 - self.step   # Switch steps

        if abs(self.cmd_fwd) < 0.05:
            self.set_state(ControlState.STAND)

    def state_yaw(self):
        # Scale step arc length (i.e., between foot touchdown and liftoff points) by forward command
        scale = self.cmd_yaw
        gnd_arc = pi/3   # Max arc length
        gnd_vel = 0.5   # Max velocity on ground
        lift_vel = 2   # Max lift velocity

        if self.step is 0:
            self.legs[LegIndex.L1].move_to(3*pi/2 - gnd_arc/2 * scale, -gnd_vel  * scale)
            self.legs[LegIndex.L2].move_to(3*pi/2 + gnd_arc/2 * scale, -lift_vel * scale)
            self.legs[LegIndex.L3].move_to(3*pi/2 - gnd_arc/2 * scale, -gnd_vel  * scale)
            self.legs[LegIndex.R1].move_to(3*pi/2 - gnd_arc/2 * scale,  lift_vel * scale)
            self.legs[LegIndex.R2].move_to(3*pi/2 + gnd_arc/2 * scale,  gnd_vel  * scale)
            self.legs[LegIndex.R3].move_to(3*pi/2 - gnd_arc/2 * scale,  lift_vel * scale)
        else:
            self.legs[LegIndex.L1].move_to(3*pi/2 + gnd_arc/2 * scale, -lift_vel * scale)
            self.legs[LegIndex.L2].move_to(3*pi/2 - gnd_arc/2 * scale, -gnd_vel  * scale)
            self.legs[LegIndex.L3].move_to(3*pi/2 + gnd_arc/2 * scale, -lift_vel * scale)
            self.legs[LegIndex.R1].move_to(3*pi/2 + gnd_arc/2 * scale,  gnd_vel  * scale)
            self.legs[LegIndex.R2].move_to(3*pi/2 - gnd_arc/2 * scale,  lift_vel * scale)
            self.legs[LegIndex.R3].move_to(3*pi/2 + gnd_arc/2 * scale,  gnd_vel  * scale)

        if all([leg.on_setpoint for leg in self.legs.values()]):
            self.step = 1 - self.step   # Switch steps

        if abs(self.cmd_fwd) > 0.05 or abs(self.cmd_yaw) < 0.05:
            self.set_state(ControlState.STAND)

    # Run controller
    def run_control(self):
        self.set_state(ControlState.ESTOP)

        state_funcs = {ControlState.ESTOP:   self.state_estop,
                       ControlState.STANDBY: self.state_standby,
                       ControlState.INIT:    self.state_init,
                       ControlState.SIT:     self.state_sit,
                       ControlState.STAND:   self.state_stand,
                       ControlState.WALK:    self.state_walk,
                       ControlState.YAW:     self.state_yaw}

        while not self.stopflag:
            state_funcs[self.state]()
            time.sleep(0.02)   # TODO(syoo): proper freq control

    def bind_socket(self, host="", port=55000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.2)

        try:
            self.sock.bind((host, port))
        except Exception as e:
            print("ERROR: {}".format(e))
            sys.exit(-1)

    # Listen for incoming command connections and data
    def run_comm(self):
        while not self.stopflag:
            self.bind_socket()
            self.sock.listen(1)
            self.set_state(ControlState.ESTOP)

            print("==> Waiting for command connection")
            while not self.stopflag:
                try:
                    conn, addr = self.sock.accept()
                    print("==> Connected!")
                    break
                except socket.timeout:
                    pass

            while not self.stopflag:
                try:
                    r = conn.recv(1024)

                    if len(r) == 0:
                        self.sock.close()
                        print("==> Client closed connection. Disconnected.")
                        break

                    if self.debug:
                        print("Received: {}".format(r))

                    cmd = json.loads(r)

                    # Sanity check
                    if abs(cmd["fwd"]) > 1.0:
                        print("ERROR: Forward velocity must be within [-1, 1]")
                    elif abs(cmd["yaw"]) > 1.0:
                        print("ERROR: Yaw velocity must be within [-1, 1]")
                    elif type(cmd["cannon"]) is not int:
                        print("ERROR: Cannon command must be a boolean")
                    elif type(cmd["estop"]) is not bool:
                        print("ERROR: Estop command must be a boolean")
                        self.cmd_estop = True   # Assume Estop if input is bad
                    else:
                        self.cmd_fwd = cmd["fwd"]
                        self.cmd_yaw = cmd["yaw"]
                        self.cmd_cannon = cmd["cannon"]
                        self.cmd_trigger = cmd["trigger"]
                        self.cmd_estop = cmd["estop"]

                    #self.print_cmds()

                    # Unconditionally set Estop if commanded
                    if self.cmd_estop:
                        self.set_state(ControlState.ESTOP)

                except socket.timeout:
                    self.sock.close()
                    print("==> Haven't received data in a while. Disconnected.")
                    break
                except socket.error as e:
                    err = e.args[0]
                    if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                        time.sleep(0.001)
                        continue
                    else:
                        self.sock.close()
                        print("ERROR: {}".format(e))
                        print("==> Disconnected")
                        break

if __name__ == "__main__":
    print("FLOWER TIME <3")
    print("")
    with open("robot_banner.txt") as f:
        for l in f.readlines():
            sys.stdout.write(l)
    print("")

    mc1 = MotorSerial("/dev/m1", 230400, 1)
    mc2 = MotorSerial("/dev/m2", 230400, 1)
    mc3 = MotorSerial("/dev/m3", 230400, 1)

    l1 = Leg("L1", Motor(mc1, 0))
    l2 = Leg("L2", Motor(mc2, 0))
    l3 = Leg("L3", Motor(mc3, 0))
    r1 = Leg("R1", Motor(mc1, 1), True)
    r2 = Leg("R2", Motor(mc2, 1), True)
    r3 = Leg("R3", Motor(mc3, 1), True)

    f = Flowergirl(l1, l2, l3, r1, r2, r3)

    print("Starting command server...")
    f.bind_socket()

    th_control = threading.Thread(target=f.run_control)
    th_comm = threading.Thread(target=f.run_comm)

    def sig_handler(signal, frame):
        f.stop()
    signal.signal(signal.SIGINT, sig_handler)

    print("Ctrl+C to stop")
    th_control.start()
    th_comm.start()

    th_control.join()
    th_comm.join()

