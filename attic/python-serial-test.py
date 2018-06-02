from __future__ import print_function
import serial
import threading
import struct
import time
import binascii

print('Connect to /dev/ttyO2')
ser_ms = serial.Serial('/dev/ttyO2', 230000, timeout=1)
fmtstr = '<BBB BBBB H BBBBBB'
fields = [81, 1, 0, 17, 17, 17, 17, 0, 0, 0, 0, 0, 0, 0]
buflen = struct.calcsize(fmtstr)
buf = struct.pack(fmtstr, *fields)
print('fields >', fields)
print('buf len', len(buf))
print('buf w>', binascii.hexlify(buf))
while True:
    print('Write to serial port')
    ser_ms.write(buf)
    print('Read from serial port')
    data = ser_ms.read(len(buf))
    print('buf r>', binascii.hexlify(data))
    time.sleep(2)
