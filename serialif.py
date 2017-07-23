import serial
import sys
import struct
import time
import csv
import binascii
from datetime import datetime

def fletcher16(dat):
	check0 = 0
	check1 = 0
	for d in dat:
		check0 = (check0 + d) % 255
		check1 = (check1 + check0) % 255
	return (check1 << 8) | check0

def appendChecksum(dat):
	cs = fletcher16(dat)
	dat.append((cs>>8)%256)
	dat.append(cs%256)
	return dat
	
class SerialTester():
	def __init__(self, port, baud, tout):
		self.s = serial.Serial(port, baud, timeout=tout)

	# Read bytes
	def readBytes(self, dataClass, dataInstruction, payloadLength):
		dat = bytearray([0x00, dataClass, dataInstruction])
		dat = appendChecksum(dat)
		self.s.write(dat)
		return self.s.read((5 + payloadLength))

	# Read unsigned uint8_t
	def readU8(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 1)
		return bytes[3]

	# Read unsigned uint16_t
	def readU16(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 2)
		return struct.unpack('>H', bytes[3:5])

	# Read signed int16_t
	def readI16(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 2)
		return struct.unpack('>h', bytes[3:5])

	# Read unsigned uint32_t
	def readU32(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 4)
		return struct.unpack('>I', bytes[3:7])

	# Read signed int32_t
	def readI32(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 4)
		return struct.unpack('>i', bytes[3:7])
	
		# Read unsigned 32
	def readF32(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 4)
		return struct.unpack('>f', bytes[3:7])
		
	# Read signed int64_t
	def readI64(self, dataClass, dataInstruction):
		bytes = self.readBytes(dataClass, dataInstruction, 8)
		return struct.unpack('>q', bytes[3:11])
	
	# Write unsigned 32
	def writeU32(self, dataClass, dataInstruction, field):
		dat = bytearray([(0x84), dataClass, dataInstruction])
		dat.append(((field >> 24) % 256))
		dat.append(((field >> 16) % 256))
		dat.append(((field >> 8 ) % 256))
		dat.append((field % 256))
		dat = appendChecksum(dat)
		self.s.write(dat)
		return self.s.read(5)
	
	# Write float32
	def writeF32(self, dataClass, dataInstruction, field):
		dat = bytearray([0x84, dataClass, dataInstruction])
		check = fletcher16(dat)
		print(check)
		dfu = bytearray(struct.pack('>f', field))
		for d in dfu:
			dat.append(d)
		dat = appendChecksum(dat)
		self.s.write(dat)
		return self.s.read(5)

def main(port): 
	stest = SerialTester(port, 230400, 1)
	stest.writeF32(0x06, 0x05, -300.0)
	while(True):
		try:
			t = time.time()
			auxPos = float(stest.readF32(0x08, 0x00)[0])
		except Exception as e:
			print(str(e))
	f.close()
	
if __name__=="__main__":
	if(len(sys.argv) < 2):
		print('No port specified')
		sys.exit()
	main(sys.argv[1])
