#!/usr/bin/env python
"""
Flowergirl cannon control
"""

import asyncio
import argparse
import asyncio
import logging
import serial
import signal
import time

import flower_log
from motor import Motor, MotorSerial

class Cannon(object):
    def __init__(self, loop, name, motor, index):
        self._loop = loop
        self._name = name
        self._motor = motor
        self._index = index
        self._stopflag = False   # Control loop stop flag. Once set, loop will terminate.

        # Logging
        self._log = logging.getLogger("{} cannon".format(self._name))
        self._log.setLevel(logging.DEBUG)
        self._log.addHandler(flower_log.ch)
        self._log.addHandler(flower_log.fh)

        self._task = self._loop.create_task(self.run())

    @property
    def task(self):
        return self._task

    def stop(self):
        """Stop control thread"""
        self._log.info("Stopping")
        self.off()
        self._motor.stop()
        self._stopflag = True

    def disable(self):
        """Disable cannon."""
        self._motor.disable(self._index)
        self._log.info("Disabled")

    def enable(self):
        """Enable cannon"""
        self._motor.enable(self._index)
        self._log.info("Enabled")

    def on(self):
        """Turn on cannon"""
        self._motor._set_cur(self._index, 0.1)   # TODO(syoo)
        self._log.info("ON <3")

    def off(self):
        """Turn off cannon"""
        self._motor._set_cur(self._index, 0)   # TODO(syoo)
        self._log.info("OFF :(")

    async def run(self):
        """Run cannon controller. This is just a dummy loop because the class was adapted from the Leg controller."""
        while not self._stopflag:
            await asyncio.sleep(0.1)

        # Disable once done running
        self.disable()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Flowergirl cannon tester')
    parser.add_argument('--dev', type=str, help='Serial device', required=True)
    parser.add_argument('--baud', type=int, help='Baudrate', default=230400)
    parser.add_argument('--index', type=int, help='Motor index', default=0)
    args = parser.parse_args()

    # Motor control
    mser = MotorSerial(args.dev, args.baud, 5)
    loop = asyncio.get_event_loop()
    m = Motor(loop, mser)

    # Cannon control
    c = Cannon(loop, "Quiet", m, args.index)
    c.enable()
    c.on()

    # Stop leg control loop on SIGINT
    signal.signal(signal.SIGINT, lambda s,f: c.stop())

    # Loop!
    loop.run_until_complete(c.task)
    loop.close()
