#-------------------------------------------------------------------------------
# File Name :
# Purpose   :
# Author    : Sri Muthu Narayanan Balasubramanian
# Created   : 17 May 2016
# Copyright :
#-------------------------------------------------------------------------------

from __future__ import division
import time
import cv2
from LaneDetect import LaneDetect
from TrafficSign import TrafficSign
from ArduinoComm import ArduinoComm
import datetime
##Picamera imports
from picamera.array import PiRGBArray
from picamera import PiCamera


DEBUG = False
VISUALIZATION = False
ACTUATION = True


##PiCamera Initialization goes here
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (800, 600)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(800, 600))

# allow the camera to warmup
time.sleep(0.1)

c_loggerFrequency = 10


c_skipSignDetection = False
c_skipLaneDetection = False
c_skipLogging = False

c_stabilizationTime = 2

c_junctionThreshLower = 125
c_junctionThreshUpper = 155
c_imminentSignThresh = 150 #in cm
c_courseCorrAngleThresh = 5


c_logFileName = "log_"+str(datetime.datetime.now())+".txt"
g_loggerCounter = 0


# 90 deg cam
#
# pixels to centroid        dist to centroid    dist to extreme         area
# 176                           43                  83                  193750
# 160                           40                  71.5                173300
# 135                           37                  57                  137560
# c_distCoeffA = -0.000725
# c_distCoeffB = 0.372
# c_distCoeffC = 0
# 175 - 85
# 150 - 64
# 125 - 51
# 100 - 43
# 50 - 33
# 0 - 26
c_pixDistThresh = 180
c_distCoeffA = 0.00274
c_distCoeffB = -0.1428
c_distCoeffC = 0
c_distLimiter = 0.45

c_cameraDefaultPan = 90
c_cameraDefaultTilt = 91
c_panAngle = 50
c_tiltAngle = 10
c_forwardPwm = 150
c_leftTurnPwm = 200
c_rightTurnPwm = 170


g_signDatabase = {"STOP":9999,"LEFT":9999,"RIGHT":9999,"STRAIGHT":9999,"UTURN":9999}
g_imminentSign = []


# cap = cv2.VideoCapture("sat_2_93d.h264")
# print "video capture opening :", cap.isOpened()

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
    return int(((distance*distance*c_distCoeffA) +
               (distance*c_distCoeffB)+c_distCoeffC)*c_distLimiter)

def StabilizationDelay(delay):
    print "sleeping"
    time.sleep(delay)

def TurnLeft(actAngle):
    if ACTUATION:
        commObj.SendMessage("ARC_LEFT", [c_leftTurnPwm, actAngle, 1, 0])
        print "TURN LEFT " + str(actAngle)
        StabilizationDelay(c_stabilizationTime)

def TurnRight(actAngle):
    if ACTUATION:
        commObj.SendMessage("ARC_RIGHT", [c_rightTurnPwm, actAngle, 1, 0])
        print "TURN RIGHT " + str(actAngle)
        StabilizationDelay(c_stabilizationTime)

def GoForward(actDist):
    if ACTUATION:
        commObj.SendMessage("FORWARD", [170, actDist, 250, 0])
        print "FORWARD " + str(actDist)+" cm"
        StabilizationDelay(c_stabilizationTime)

def GoReverse(actDist):
    if ACTUATION:
        commObj.SendMessage("REVERSE", [c_forwardPwm, actDist, 1, 0])
        print "too close to junction: REVERSE " + str(actDist) + " cm"
        StabilizationDelay(c_stabilizationTime)

def NoSignTurnHandler(ccAngle):

    rawCapture.truncate(0)
    SetCameraPosition(c_cameraDefaultPan+c_panAngle,c_cameraDefaultTilt-c_tiltAngle)
    camera.capture(rawCapture, format="bgr")
    StabilizationDelay(c_stabilizationTime)
    imgL = rawCapture.array
    lAng,lDist,lArea = laneObj.DetectLane(imgL)

    SetCameraPosition(c_cameraDefaultPan,c_cameraDefaultTilt)

    rawCapture.truncate(0)
    SetCameraPosition(c_cameraDefaultPan-c_panAngle,c_cameraDefaultTilt-c_tiltAngle)
    camera.capture(rawCapture, format="bgr")
    StabilizationDelay(c_stabilizationTime)
    imgR = rawCapture.array
    rAng,rDist,rArea = laneObj.DetectLane(imgR)

    SetCameraPosition(c_cameraDefaultPan,c_cameraDefaultTilt)

    if lDist>rDist:
        actAngle = 90-ccAngle
        TurnLeft(actAngle)
    else:
        actAngle = 90+ccAngle
        TurnRight(actAngle)


def JunctionHandler(ccAngle,ccDistance):

    if IsJunctionTooClose(ccDistance):
        avg = int((c_junctionThreshLower+c_junctionThreshUpper)/2)
        actDist = GetPhysicalDistance(avg-ccDistance)
        GoReverse(actDist)

    if g_imminentSign:
        #Imminent sign exists
        if g_imminentSign[0]=="LEFT":
            actAngle = 90-ccAngle
            TurnLeft(actAngle)
            
            PopCompletedSign("LEFT")
            return 0
        elif g_imminentSign[0] == "RIGHT":
            actAngle = 90+ccAngle
            TurnRight(actAngle)
            
            PopCompletedSign("RIGHT")
            return 0
        elif g_imminentSign[0] == "STOP":
            actAngle = 90 + ccAngle
            if ACTUATION:
                commObj.SendMessage("STOP", [200, 200, 1, 0])
                print "STOPPING "
                StabilizationDelay(c_stabilizationTime)
            
            PopCompletedSign("STOP")
            return 1
        elif g_imminentSign[0] == "UTURN":
            actAngle = 180+ccAngle
            TurnRight(actAngle)
            
            PopCompletedSign("UTURN")
            return 0
    else:
        NoSignTurnHandler(ccAngle)
        return 0

def AngleCourseCorrection(ccAngle):

    if ccAngle>0:
        TurnLeft(ccAngle)
        
    else:
        TurnRight(ccAngle)
        


def DistanceCourseCorrection(ccDistance):
    actDist = GetPhysicalDistance(ccDistance)
    GoForward(actDist)
    
    UpdateSignDistances(actDist)


def NonJunctionHandler(ccAngle,ccDistance):
    if IsCcAngleAboveThresh(ccAngle):
        AngleCourseCorrection(ccAngle)

    DistanceCourseCorrection(ccDistance)
    return 0

def TerminateAll():
    if ACTUATION:
        c_logFile.close()

def ControlMain(img_orig):

    print "Control Main"

    if ACTUATION:
        UpdateLoggerCount()
        if not c_skipLogging:
            c_logFile.write(commObj.ReceiveMessage())

    if not c_skipSignDetection:
        signsDict = signObj.DetectSign(img_orig)
        print signsDict
        print g_signDatabase
        print g_imminentSign
        UpdateSignDatabase(signsDict)
    if not c_skipLaneDetection:
        ccAngle, ccDistance, ccArea = laneObj.DetectLane(img_orig)
        print ccAngle,ccDistance,ccArea
        if not IsCcDistanceValid(ccDistance):
            return 0
        if IsJunction(ccDistance):
            return JunctionHandler(ccAngle,ccDistance)
        else:
            return NonJunctionHandler(ccAngle,ccDistance)


def SetCameraPosition(pan,tilt):
    if ACTUATION:
        commObj.SendMessage("PAN_SERVO",[pan,0,0,0])
        time.sleep(0.5)
        commObj.SendMessage("TILT_SERVO", [tilt, 0, 0, 0])
        StabilizationDelay(c_stabilizationTime)
        
##Main function for running in windows
# if __name__ == '__main__':
#
#     while(cap.isOpened()):
#         ret, img_orig = cap.read()
#         terminate = ControlMain(img_orig)
#         if(terminate):
#             TerminateAll()
#             break
#         if DEBUG:
#             cv2.waitKey(0)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break
#     cap.release()
#     cv2.destroyAllWindows()

##Main function for the DEMO

if __name__ == '__main__':
    SetCameraPosition(c_cameraDefaultPan,c_cameraDefaultTilt)
    #comm = ArduinoComm.ArduinoComm("/dev/ttyUSB0",9600)
    terminate = 0
    while not terminate:
        #time.sleep(0.05)
        camera.capture(rawCapture, format="bgr")
        img_orig = rawCapture.array
        #print "img captured"
        terminate = ControlMain(img_orig)
     
        rawCapture.truncate(0)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        #cv2.waitKey(0)


    TerminateAll()
        
        


        
