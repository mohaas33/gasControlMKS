#!/usr/bin/python3
import time
import serial

def _readline(ser):
    eol = b'\r'
    leneol = len(eol)
    line = bytearray()
    while True:
        c = ser.read(1)
        if c:
            line += c
            if line[-leneol:] == eol:
                break
        else:
            break
    return bytes(line)
# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()
# Reading the data from the serial port. This will be running in an infinite loop.
# print ('Enter your commands below.\r\nInsert "exit" to leave the application.')
inpt = b"ID\r"
ser.write(inpt)
print("response: '{}'".format(_readline(ser)))

