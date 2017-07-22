#!/usr/bin/env python
"""
FLOWERGIRL
"""

import enum
import signal
import threading
import time

# Left and right motors, #1 through 3 front to rear
class Motor(enum.Enum):
    L1 = 0
    L2 = 1
    L3 = 2
    R1 = 3
    R2 = 4
    R3 = 5

class Flowergirl(object):
    def __init__(self):
        self.fwd_vel = 0.0   # [-1, 1]
        self.yaw_vel = 0.0   # [-1, 1]
        self.cannon  = False
        self.stopflag = False

        self.motor_values = [0.0]*6   # [-1, 1]

    def stop(self):
        print("no more flowers :(")
        self.stopflag = True

    def print_state(self):
        print("Forward: {0:1.1f}   Yaw: {1:1.1f}   Cannon: {2:}".format(self.fwd_vel, self.yaw_vel, "On" if self.cannon else "Off"))

    # All desired motor values should be set together by the controller
    def set_motors(self, l1, l2, l3, r1, r2, r3):
        self.motor_values[Motor.L1] = l1
        self.motor_values[Motor.L2] = l2
        self.motor_values[Motor.L3] = l3
        self.motor_values[Motor.R1] = r1
        self.motor_values[Motor.R2] = r2
        self.motor_values[Motor.R3] = r3

    # Motor commands are sent at a fixed frequency by the controller
    def _command_motor(self, index):
        pass

    def set_actuators(self, fwd_vel, yaw_vel, cannon):
        # Sanity check
        if abs(fwd_vel) > 1.0:
            print("ERROR: Forward velocity must be within [-1, 1]")
            return
        elif abs(yaw_vel) > 1.0:
            print("ERROR: Yaw velocity must be within [-1, 1]")
            return
        elif type(cannon) is not bool:
            print("ERROR: Cannon command must be a boolean")
            return

        self.fwd_vel = fwd_vel
        self.yaw_vel = yaw_vel
        self.cannon  = cannon

    def run(self):
        while not self.stopflag:
            self.print_state()
            time.sleep(0.2)

if __name__ == "__main__":
    print("FLOWERS EVERYWHERE! <3")
    print("")
    print("               ____          ___           __----__   _/\\")
    print("            _/^ __ ^\\_    /~^_/ |       )/^        ^-^ _/")
    print("         _/^ _/^  ^\\_ ^\\ | ./  /~      /(            _/\\.")
    print("       _/^_/^--_     ^\\_^\\-__-~      _/( \\         _/  ./")
    print("     ./^_/|  \\_ ~\\      \\_^\\_      /^ _(  ~-_    _/ \\./\\")
    print("   _/^_/   \\_  ~\\ \\      ^\\__^\\../^_/^ )\\    ~~~~    _\\/")
    print("  <__/       ~\\__\\|         ^\\.__./^      ~---____--~ ~\\")
    print("")

    f = Flowergirl()
    th_control = threading.Thread(target=f.run)

    def sig_handler(signal, frame):
        f.stop()
    signal.signal(signal.SIGINT, sig_handler)

    print("Ctrl+C to stop")
    th_control.start()
    th_control.join()

