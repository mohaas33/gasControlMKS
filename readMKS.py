#!/usr/bin/python3
import time
import serial
#import sys


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


def readMKS(inputLine = 'ID'):
    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=9600,
        parity=serial.PARITY_ODD,
        bytesize=serial.EIGHTBITS
    )
    ser.isOpen()
    # Reading the data from the serial port. This will be running in an infinite loop.
    # print ('Enter your commands below.\r\nInsert "exit" to leave the application.')
    
    #s = 0
    #s = 1..5
    #GM R Select gas menu
    #gas menu X, normal setpoints are used
    #gas menu 1-5
    #check for gas menu, result: s
    
    #inpt = b"GM 2 R\r" #check for gas menu
    #inpt = b"FL 1 \r"  #check flow of a channel
    #inpt = b"PR\r"     #check pressure
    #inpt = b"PM R\r"   #check for pressure mode
    #inpt = b"ID\r"
    
    #list_of_arguments = ''.join(sys.argv[1:])
    inpt = str.encode('{}\r'.format(inputLine))
    ser.write(inpt)
    return '{}'.format(_readline(ser))

