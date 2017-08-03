#!/usr/bin/env python
"""
Spam Flowergirl with control inputs derived from joystick values
"""

import evdev
import json
import signal
import socket
import sys
import threading
import time

class JoyValue(object):
    def __init__(self, name, value, offset=None, scale=None):
        self.name = name
        self.value = value
        self.offset = offset
        self.scale = scale

    def set(self, raw_value):
        if not self.offset or not self.scale:
            self.value = raw_value
        else:
            self.value = (float(raw_value) + self.offset) * self.scale

class Joystick(object):
    def __init__(self, debug=False):
        self.stopflag = False
        self.device = self.get_device()
        self.debug = debug

        # Comm socket
        self.sock = None

        # Values buffer
        self.x = JoyValue('stick_x', 0.0, -512, 1./512)
        self.y = JoyValue('stick_y', 0.0, -512, -1./512)
        self.z = JoyValue('stick_z', 0.0, -128, -1./128)
        self.throttle = JoyValue('throttle', 0.0, -255, -1./255)
        self.trigger = JoyValue('trigger', 0)
        self.thumb = JoyValue('thumb', 0)
        self.hat_y = JoyValue('hat_y', 0)

        # Event codes
        self.events = {0: self.x,
                       1: self.y,
                       2: self.throttle,
                       5: self.z,
                       17: self.hat_y,
                       288: self.trigger,
                       289: self.thumb}

    def stop(self):
        print("no more flowers :(")
        self.stopflag = True

    def print_state(self):
        print("X: {:+1.2f} Y: {:+1.2f} Z: {:+1.2f} Throttle: {:+1.2f} Trig: {} Thumb: {} HatY: {}".format(self.x.value, self.y.value, self.z.value, self.throttle.value, self.trigger.value, self.thumb.value, self.hat_y.value))

    def get_device(self):
        devs = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
        if len(devs) == 0:
            print('Could not find a joystick. Exiting.')
            sys.exit(-1)

        # Just grab the first one.
        dev = devs[0]
        print("Found joystick:")
        print("  - File: {}".format(dev.fn))
        print("  - Name: {}".format(dev.name))
        print("  - Phys: {}".format(dev.phys))
        print("Capabilities: {}".format(dev.capabilities()))

        return dev

    # Handle joystick events
    def run_joystick(self):
        while not self.stopflag:
            try:
                for ev in self.device.read():
                    if not ev.type == evdev.ecodes.EV_KEY and not ev.type == evdev.ecodes.EV_ABS:
                        continue   # Not sure what to do with this
                    if self.events.get(ev.code):
                        self.events[ev.code].set(ev.value)
                        if self.debug:
                            print("Event: {}   val: {}".format(self.events[ev.code].name, ev.value))
                        else:
                            self.print_state()
                    elif self.debug:
                        print("Unknown event code: {}   val: {}".format(ev.code, ev.value))
            except BlockingIOError as e:
                time.sleep(0.001)

    def connect(self, host="flowergirl", port=55000):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(0.1)

        try:
            self.sock.connect((host, port))
        except Exception as e:
            print("ERROR: {}".format(e))
            sys.exit(-1)

    # Spam the flowergirl with command values
    def run_comm(self):
        while not self.stopflag:
            cmd = json.dumps({"fwd": self.y.value, "yaw": -self.x.value, "cannon": self.thumb.value, "trigger": self.trigger.value, "hat_y": -self.hat_y.value, "estop": True if self.throttle.value < 0.8 else False}).encode()
            try:
                s = self.sock.sendall(cmd)
            except ConnectionResetError:
                print("Lost connection. Exiting.")
                self.stop()
                break
            time.sleep(0.01)

        self.sock.close()

if __name__ == "__main__":
    print("FLOWER POWER! <3")
    print("")
    with open("js_banner.txt") as f:
        for l in f.readlines():
            sys.stdout.write(l)
    print("")

    j = Joystick()

    print("Connecting to Flowergirl...")
    j.connect()

    th_joystick = threading.Thread(target=j.run_joystick)
    th_comm = threading.Thread(target=j.run_comm)

    def sig_handler(signal, frame):
        j.stop()
    signal.signal(signal.SIGINT, sig_handler)

    print("Ctrl+C to stop")
    th_joystick.start()
    th_comm.start()

    th_joystick.join()
    th_comm.join()

