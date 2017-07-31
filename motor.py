#!/usr/bin/env python
"""
Flowergirl motor control
"""

import serial
import sys
import struct
import time
import csv
import binascii
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

    # Read bytes
    def read_bytes(self, data_class, data_inst, payload_len):
        dat = bytearray([0x00, data_class, data_inst])
        dat = append_checksum(dat)
        print("Wrote {} bytes".format(len(dat)))
        self.ser.write(dat)
        print("Reading...")
        r = self.ser.read(5 + payload_len)

        if len(r) != 5 + payload_len:
            raise RuntimeError("Expected {} bytes but only read {}".format(5+payload_len, len(r)))

        return r

    # Read unsigned uint8_t
    def read_u8(self, data_class, data_inst):
        r = self.read_bytes(data_class, data_inst, 1)
        return bytes[3]

    # Read unsigned uint16_t
    def read_u16(self, data_class, data_inst):
        bytes = self.read_bytes(data_class, data_inst, 2)
        return struct.unpack('>H', bytes[3:5])

    # Read signed int16_t
    def read_i16(self, data_class, data_inst):
        bytes = self.read_bytes(data_class, data_inst, 2)
        return struct.unpack('>h', bytes[3:5])

    # Read unsigned uint32_t
    def read_u32(self, data_class, data_inst):
        bytes = self.read_bytes(data_class, data_inst, 4)
        return struct.unpack('>I', bytes[3:7])

    # Read signed int32_t
    def read_i32(self, data_class, data_inst):
        bytes = self.read_bytes(data_class, data_inst, 4)
        return struct.unpack('>i', bytes[3:7])

        # Read unsigned 32
    def read_f32(self, data_class, data_inst):
        bytes = self.read_bytes(data_class, data_inst, 4)
        return struct.unpack('>f', bytes[3:7])

    # Read signed int64_t
    def read_i64(self, data_class, data_inst):
        bytes = self.read_bytes(data_class, data_inst, 8)
        return struct.unpack('>q', bytes[3:11])

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
        print("[{:.3f}] Checksum: {}".format(time.time(), check))
        dfu = bytearray(struct.pack('>f', field))
        for d in dfu:
            dat.append(d)
        dat = append_checksum(dat)
        self.ser.write(dat)
        return self.ser.read(5)

class Motor(object):
    def __init__(self, mser, index):
        self._mser = mser   # MotorSerial instance
        self._index = index   # Each controller controls two motors,

    def estop(self):
        self._mser.write_u32(SYSTEM_STATE_BASE, SYS_OUTPUT_ENABLE, 0)
        # TODO(syoo): reenable?

    def get_cur(self):
        """Get current in amps"""
        return self._mser.read_f32(CURRENT_BASE, I_MEASURED)

    def get_vel(self):
        """Get velocity in radians/second"""
        return self._mser.read_f32(V_BASE, V_MEASURED)

    def get_pos(self):
        """Get position in radians"""
        return self._mser.read_f32(P_BASE, P_MEASURED)

    def set_cur(self, cur):
        """Set current in amps"""
        return self._mser.write_f32(CURRENT_BASE, I_MEASURED, cur)

    def set_vel(self, vel):
        """Set velocity in radians/second"""
        return self._mser.write_f32(V_BASE, V_MEASURED, vel)

    def set_pos(self, pos):
        """Set position in radians"""
        return self._mser.write_f32(P_BASE, P_MEASURED, pos)

def main(port):
    stest = MotorSerial(port, 460800, 0.1)

    while True:
        try:
            t = time.time()
            pos = float(stest.read_f32(SYSTEM_STATE_BASE, SYS_DRIVE_STATUS)[0])
        except Exception as e:
            print(str(e))

    f.close()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("No port specified")
        sys.exit()

    main(sys.argv[1])
