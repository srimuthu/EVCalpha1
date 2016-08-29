#-------------------------------------------------------------------------------
# Name:        ArduinoComm.py
# Purpose:      Contains definitions for messaging conventions between
#               Pi and Arduino
#
# Author:      Sri Muthu Narayanan Balasubramanian
#
# Created:     17-05-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import sys, time, os
import serial
from Error import *

msgLen  = 5
serPort = "/dev/ttyUSB0"
cmdDict = {"NO_PENDING":0xfe,
            "BLINK_LED":0x01,
            "FORWARD":0x02,
            "REVERSE":0x03,
            "STOP":0x04,
            "ARC_LEFT":0x05,
           "ARC_RIGHT":0x06,
           "PAN_SERVO":0x07,
           "TILT_SERVO":0x08}

class ArduinoComm():
    def __init__(self, port=serPort, baud=9600):
        self.__InitSuccess = False
        try:
            self.__ser = serial.Serial(str(port),baud)
            print Error.serSuccess
            self.__InitSuccess = True
            time.sleep(2)

        except:
            sys.exit(Error.serInitError)

        self.__cmd = chr(cmdDict["NO_PENDING"])
        self.__msg = [self.__cmd,chr(0x00),chr(0x00),chr(0x00),chr(0x00)]

    def __del__(self):
        self.__ser.close()
        print Error.serClose

    def Terminate(self):
        self.__del__()


    def __TransmitMessage(self):

        for i in range(0,msgLen):
            self.__ser.write(self.__msg[i])
        print "Transmitting :",self.__msg

    def SendMessage(self, cmd, args):
        if cmd not in cmdDict:
            print Error.invalidCmd
            return False

        self.__cmd =chr(cmdDict[cmd])
        self.__msg[0] = self.__cmd
        for i in range(1,msgLen):
            self.__msg[i] = chr(args[i-1])

        self.__TransmitMessage()


#testing purposes
if __name__ == '__main__':

    comm = ArduinoComm("/dev/ttyUSB0",9600)
    #comm.SendMessage("BLINK_LED",[25,250,0,0])
    #time.sleep(10)
    comm.SendMessage("REVERSE",[210,100,1,0])
    time.sleep(10)
    comm.SendMessage("STOP",[0,0,0,0])
    time.sleep(5)
    comm.SendMessage("FORWARD",[210,100,1,0])
    time.sleep(10)
    comm.SendMessage("STOP",[0,0,0,0])
    time.sleep(5)
    comm.Terminate()
