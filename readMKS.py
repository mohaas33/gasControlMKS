#!/usr/bin/python3
import time
import serial

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB1',
    baudrate=9600,
    timeout=1,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()
# Reading the data from the serial port. This will be running in an infinite loop.
print ('Enter your commands below.\r\nInsert "exit" to leave the application.')

inpt=1
while 1 :
    # get keyboard input
    inpt = input(">> ")
    if inpt == 'exit':
        ser.close()
        exit()
    else:
        # send the character to the device
        # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
        #ser.write(inpt + '\r\n')
        ser.write(str.encode(inpt))
        out = ''
        # let's wait one second before reading output (let's give device time to answer)
        time.sleep(1)
        while ser.inWaiting() > 0:
            #line = ser.read(1024)  
            #print(ser.read(2048))
            out += str(ser.read(1))
            #print(out)

        if out != '':
            print ("Output >>" + out)
