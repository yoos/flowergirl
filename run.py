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

# Left and right motors, #1 through 3 front to rear
class Motor(enum.Enum):
    L1 = 0
    L2 = 1
    L3 = 2
    R1 = 3
    R2 = 4
    R3 = 5

class Flowergirl(object):
    def __init__(self, debug=False):
        self.fwd_vel = 0.0   # [-1, 1]
        self.yaw_vel = 0.0   # [-1, 1]
        self.cannon  = False
        self.stopflag = False
        self.debug = debug

        # Comm socket
        self.sock = None

        self.motor_values = [0.0]*6   # [-1, 1]

    def stop(self):
        print("no more flowers :(")
        self.stopflag = True

    def print_state(self):
        print("Forward: {:+1.2f}   Yaw: {:+1.2f}   Cannon: {}".format(self.fwd_vel, self.yaw_vel, "On" if self.cannon else "Off"))

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

    def run_control(self):
        while not self.stopflag:
            # TODO(syoo): fwd/yaw to rhex gait control
            self.print_state()
            time.sleep(0.025)

    def disarm(self):
        self.fwd_vel = 0.0
        self.yaw_vel = 0.0
        self.cannon = 0

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
            self.disarm()

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
                    self.fwd_vel = cmd["fwd"]
                    self.yaw_vel = cmd["yaw"]
                    self.cannon = cmd["cannon"]

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

    f = Flowergirl()

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

