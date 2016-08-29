#-------------------------------------------------------------------------------
# Name:        EVC_main.py
# Purpose:     Main Lane Detection module
#
# Author:      Sri Muthu Narayanan Balasubramanian
#
# Created:     17-05-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------
from __future__ import division
import time
import cv2
from LaneDetect import LaneDetect
from TrafficSign import TrafficSign
from ArduinoComm import ArduinoComm
import datetime
##Picamera imports
# from picamera.array import PiRGBArray
# from picamera import PiCamera


DEBUG = False
VISUALIZATION = True
ACTUATION = True


##PiCamera Initialization goes here
# initialize the camera and grab a reference to the raw camera capture
# camera = PiCamera()
# camera.resolution = (800, 600)
# camera.framerate = 32
# rawCapture = PiRGBArray(camera, size=(800, 600))

# # allow the camera to warmup
# time.sleep(0.1)

c_loggerFrequency = 10


c_skipSignDetection = False
c_skipLaneDetection = False
c_skipLogging = False

c_stabilizationTime = 3

c_junctionThreshLower = 125
c_junctionThreshUpper = 140
c_imminentSignThresh = 150 #in cm
c_courseCorrAngleThresh = 10


c_logFileName = "log_"+str(datetime.datetime.now())+".txt"
g_loggerCounter = 0


# 90 deg cam
#
# pixels to centroid		dist to centroid	dist to extreme	        area
# 176					        43					83					193750
# 160					        40					71.5				173300
# 135					        37					57					137560
c_pixDistThresh = 180
c_distCoeffA = -0.000725
c_distCoeffB = 0.372
c_distCoeffC = 0

g_signDatabase = {"STOP":9999,"LEFT":9999,"RIGHT":9999,"STRAIGHT":9999,"UTURN":9999}
g_imminentSign = []


cap = cv2.VideoCapture("sat_2_93d.h264")
print "video capture opening :", cap.isOpened()

signObj = TrafficSign()
laneObj = LaneDetect()

if ACTUATION:
    #commObj = ArduinoComm("COM3")
    commObj = ArduinoComm("/dev/ttyUSB0", 9600)
    c_logFile = open(c_logFileName,'w')


def IsJunction(distance):
    if distance < c_junctionThreshUpper:
        return True
    else:
        return False

def IsJunctionTooClose(distance):
    if distance < c_junctionThreshLower:
        return True
    else:
        return False

def IsCcAngleAboveThresh(angle):
    if angle > c_courseCorrAngleThresh:
        return True
    else:
        return False

def IsCcDistanceValid(distance):
    if distance < c_pixDistThresh:
        return True
    else:
        return False

def UpdateLoggerCount():
    global g_loggerCounter
    g_loggerCounter= g_loggerCounter + 1
    if g_loggerCounter>c_loggerFrequency:
        c_skipLogging = False
        g_loggerCounter = 0
    else:
        c_skipLogging = True

def UpdateSignDatabase(signsDict={}):

    global g_signDatabase

    #if TSD returns some signs, update them
    if signsDict:
        for gKey in g_signDatabase.keys():
            for lKey in signsDict.keys():
                if gKey == lKey:
                    if signsDict[lKey]<=g_signDatabase[gKey]:
                        g_signDatabase[gKey] = signsDict[lKey]
    UpdateImminentSign()


def PopCompletedSign(sign):
    global g_imminentSign
    global g_signDatabase
    g_imminentSign = []
    g_signDatabase[sign]=9999

def UpdateImminentSign():

    global g_imminentSign
    #Find the imminent sign
    for sign in g_signDatabase.keys():
        if g_signDatabase[sign] < c_imminentSignThresh:
            if not g_imminentSign:
                # Imminent sign is not occupied
                g_imminentSign = [str(sign)]

def UpdateSignDistances(distance):
    global g_signDatabase
    for sign in g_signDatabase.keys():
        if g_signDatabase[sign]!=9999:
            g_signDatabase[sign] -= distance
    UpdateImminentSign()

def GetPhysicalDistance(distance):
    return int((distance*distance*c_distCoeffA) +
               (distance*c_distCoeffB)+c_distCoeffC)

def StabilizationDelay(delay):
    time.sleep(delay)

def JunctionHandler(ccAngle,ccDistance):

    if IsJunctionTooClose(ccDistance):
        avg = int((c_junctionThreshLower+c_junctionThreshUpper)/2)
        actDist = GetPhysicalDistance(avg-ccDistance)
        if ACTUATION:
            commObj.SendMessage("REVERSE",[170,actDist,1,0])
            StabilizationDelay(c_stabilizationTime)
        print "too close to junction: REVERSE "+str(actDist)+" cm"

    if g_imminentSign:
        #Imminent sign exists
        if g_imminentSign[0]=="LEFT":
            actAngle = 90-ccAngle
            if ACTUATION:
                commObj.SendMessage("ARC_LEFT",[200,actAngle,1,0])
                StabilizationDelay(c_stabilizationTime)
            print "TURN LEFT "+str(actAngle)
            PopCompletedSign("LEFT")
            return 0
        elif g_imminentSign[0] == "RIGHT":
            actAngle = 90+ccAngle
            if ACTUATION:
                commObj.SendMessage("ARC_RIGHT",[170,actAngle,1,0])
                StabilizationDelay(c_stabilizationTime)
            print "TURN RIGHT "+str(actAngle)
            PopCompletedSign("RIGHT")
            return 0
        elif g_imminentSign[0] == "STOP":
            actAngle = 90 + ccAngle
            if ACTUATION:
                commObj.SendMessage("STOP", [200, 200, 1, 0])
                StabilizationDelay(c_stabilizationTime)
            print "STOPPING "
            PopCompletedSign("STOP")
            return 1
        elif g_imminentSign[0] == "UTURN":
            actAngle = 180+ccAngle
            if ACTUATION:
                commObj.SendMessage("ARC_RIGHT",[170,actAngle,1,0])
                StabilizationDelay(c_stabilizationTime)
            print "UTURN"
            PopCompletedSign("UTURN")
    else:
        actAngle = 90 - ccAngle
        if ACTUATION:
            commObj.SendMessage("ARC_LEFT", [200, actAngle, 1, 0])
            StabilizationDelay(c_stabilizationTime)
        print "TURN LEFT " + str(actAngle)
        return 0

def AngleCourseCorrection(ccAngle):

    if ccAngle>0:
        if ACTUATION:
            commObj.SendMessage("ARC_LEFT", [200, ccAngle, 1, 0])
            StabilizationDelay(c_stabilizationTime)
        print "Adjust LEFT " + str(ccAngle)
    else:
        if ACTUATION:
            commObj.SendMessage("ARC_RIGHT", [170, ccAngle, 1, 0])
            StabilizationDelay(c_stabilizationTime)
        print "Adjust RIGHT " + str(ccAngle)


def DistanceCourseCorrection(ccDistance):
    actDist = GetPhysicalDistance(ccDistance)
    if ACTUATION:
        commObj.SendMessage("FORWARD", [200, actDist, 250, 0])
        StabilizationDelay(c_stabilizationTime)
    print "FORWARD " + str(actDist)+" cm"
    UpdateSignDistances(ccDistance)


def NonJunctionHandler(ccAngle,ccDistance):
    if IsCcAngleAboveThresh(ccAngle):
        AngleCourseCorrection(ccAngle)

    DistanceCourseCorrection(ccDistance)

def TerminateAll():
    if ACTUATION:
        c_logFile.close()

def ControlMain(img_orig):

    actuationAngle = 0
    actuationDistance = 0

    if not c_skipSignDetection:
        signsDict = signObj.DetectSign(img_orig)
        UpdateSignDatabase(signsDict)
    if not c_skipLaneDetection:
        ccAngle, ccDistance, ccArea = laneObj.DetectLane(img_orig)
        if not IsCcDistanceValid(ccDistance):
            return 0
        if IsJunction(ccDistance):
            JunctionHandler(ccAngle,ccDistance)
        else:
            NonJunctionHandler(ccAngle,ccDistance)

    if ACTUATION:
        UpdateLoggerCount()
        if not c_skipLogging:
            c_logFile.write(commObj.ReceiveMessage())


def SetCameraPosition():
    if ACTUATION:
        commObj.SendMessage("PAN_SERVO",[90,0,0,0])
        time.sleep(0.5)
        commObj.SendMessage("TILT_SERVO", [90, 0, 0, 0])
        StabilizationDelay(c_stabilizationTime)

##Main function for running in windows
if __name__ == '__main__':

    while(cap.isOpened()):
        ret, img_orig = cap.read()
        terminate = ControlMain(img_orig)
        if(terminate):
            TerminateAll()
            break
        if DEBUG:
            cv2.waitKey(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

##Main function for the DEMO
# if __name__ == '__main__':

#     SetCameraPosition()
#     for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
#         #time.sleep(0.05)
#         img_orig = frame.array
#         terminate = ControlMain(img_orig)
#         if(terminate):
#             break
#     key = cv2.waitKey(1) & 0xFF

#     # clear the stream in preparation for the next frame
# 	rawCapture.truncate(0)

#     # if the `q` key was pressed, break from the loop
# 	if key == ord("q"):
#         TerminateAll()
#         break