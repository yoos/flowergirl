#!/usr/bin/env python
"""
Flowergirl motor control
"""

import argparse
import asyncio
import binascii
import csv
import serial
import struct
import sys
import time
from datetime import datetime

from serial_defines import *

def fletcher16(dat):
    check0 = 0
    check1 = 0
    for d in dat:
        check0 = (check0 + d) % 255
        check1 = (check1 + check0) % 255
    return (check1 << 8) | check0

def append_checksum(dat):
    cs = fletcher16(dat)
    dat.append((cs>>8)%256)
    dat.append(cs%256)
    return dat

class MotorSerial(object):
    def __init__(self, port, baud, tout=1):
        self.ser = serial.Serial(port, baud, timeout=tout)

    def read_bytes(self, data_class, data_inst, payload_len):
        dat = bytearray([0x00, data_class, data_inst])
        dat = append_checksum(dat)

        w = self.ser.write(dat)
        if w != len(dat):
            raise RuntimeError("Expected to write {} bytes but only wrote {}".format(len(dat), w))

        r = self.ser.read(5 + payload_len)
        if len(r) != 5 + payload_len:
            raise RuntimeError("Expected {} bytes but only read {}".format(5+payload_len, len(r)))

        return r

    # Read unsigned uint8_t
    def read_u8(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 1)
        return r[3]

    # Read unsigned uint16_t
    def read_u16(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 2)
        return struct.unpack('>H', r[3:5])[0]

    # Read signed int16_t
    def read_i16(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 2)
        return struct.unpack('>h', r[3:5])[0]

    # Read unsigned uint32_t
    def read_u32(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 4)
        return struct.unpack('>I', r[3:7])[0]

    # Read signed int32_t
    def read_i32(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 4)
        return struct.unpack('>i', r[3:7])[0]

    # Read unsigned 32
    def read_f32(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 4)
        return struct.unpack('>f', r[3:7])[0]

    # Read signed int64_t
    def read_i64(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 8)
        return struct.unpack('>q', r[3:11])[0]

    # Write unsigned 32
    def write_u32(self, data_class, data_inst, field):
        dat = bytearray([(0x84), data_class, data_inst])
        dat.append(((field >> 24) % 256))
        dat.append(((field >> 16) % 256))
        dat.append(((field >> 8 ) % 256))
        dat.append((field % 256))
        dat = append_checksum(dat)
        self.ser.write(dat)
        return self.ser.read(5)

    # Write float32
    def write_f32(self, data_class, data_inst, field):
        dat = bytearray([0x84, data_class, data_inst])
        check = fletcher16(dat)
        dfu = bytearray(struct.pack('>f', field))
        for d in dfu:
            dat.append(d)
        dat = append_checksum(dat)
        self.ser.write(dat)
        return self.ser.read(5)

class Motor(object):
    def __init__(self, loop, mser, index):
        self.loop = loop
        self._mser = mser   # MotorSerial instance
        self._index = index   # Each controller controls two motors,
        self.stopflag = False

        self.read_requests = []
        self.vel_sp = 0

        self.cur = 0
        self.vel = 0
        self.pos = 0

    def stop(self):
        self.stopflag = True

    def disable(self):
        self._disable()

    def enable(self):
        self._enable()

    def set_vel_sp(self, vel):
        self.vel_sp = vel

    async def read(self):
        self._mser.ser.flush()

        last_update = time.time()
        update_count = 0
        while not self.stopflag:
            now = time.time()
            update_count += 1
            if now - last_update > 1:
                print("[{:.3f}] Cur: {:.3f} Vel: {:.3f} Pos: {:.3f}   (Refresh: {} Hz)".format(now, self.cur, self.vel, self.pos, update_count))
                last_update = now
                update_count = 0

            # Special requests
            for req in self.read_requests:
                await self.loop.run_in_executor(None, req)

            # Always poll these
            self.cur = await self.loop.run_in_executor(None, self._get_cur)
            self.vel = await self.loop.run_in_executor(None, self._get_vel)
            self.pos = await self.loop.run_in_executor(None, self._get_pos)

            # Always command velocity
            r = await self.loop.run_in_executor(None, self._set_vel, self.vel_sp)
            if r[0] == '\xe0':
                print("error: {}".format(r))

    def _disable(self):
        self._mser.write_u32(SYSTEM_STATE_BASE, SYS_OUTPUT_ENABLE, 0)
        self.vel_sp = 0

    def _enable(self):
        self._mser.write_u32(SYSTEM_STATE_BASE, SYS_OUTPUT_ENABLE, 1)

    def _get_cur(self):
        """Get current in amps"""
        return self._mser.read_f32(CURRENT_BASE, I_MEASURED)

    def _get_vel(self):
        """Get velocity in radians/second"""
        return self._mser.read_f32(V_BASE, V_MEASURED)

    def _get_pos(self):
        """Get position in radians"""
        return self._mser.read_f32(P_BASE, P_MEASURED)

    def _set_cur(self, cur):
        """Set current in amps"""
        return self._mser.write_f32(CURRENT_BASE, I_REF, cur)

    def _set_vel(self, vel):
        """Set velocity in radians/second"""
        return self._mser.write_f32(V_BASE, V_REF, vel)

    def _set_pos(self, pos):
        """Set position in radians"""
        return self._mser.write_f32(P_BASE, P_REF, pos)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Flowergirl motor tester')
    parser.add_argument('--dev', type=str, help='Serial device', required=True)
    parser.add_argument('--baud', type=int, help='Baudrate', default=921600)
    parser.add_argument('--index', type=int, help='Motor index', default=0)
    parser.add_argument('--cmd', type=str, help='Single motor command test', default='_get_cur')
    parser.add_argument('--cmdarg', type=float, help='Single motor command arg')
    parser.add_argument('--continuous', help='Continuous state update test', action='store_true')

    args = parser.parse_args()

    mser = MotorSerial(args.dev, args.baud, 5)
    loop = asyncio.get_event_loop()
    m = Motor(loop, mser, args.index)

    mfunc = None
    try:
        mfunc = getattr(m, args.cmd)
    except AttributeError:
        print("{} does not implement {}".format(type(m).__name__, args.cmd))
        exit(1)

    if not args.continuous:
        r = mfunc(args.cmdarg) if args.cmdarg else mfunc()
        print("Read: {}".format(r))
    else:
        loop.run_until_complete(m.read())
        loop.close()
