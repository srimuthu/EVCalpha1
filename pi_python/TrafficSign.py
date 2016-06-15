#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      User
#
# Created:     05-06-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Name:        TrafficSign.py
# Purpose:
#
# Author:      User
#
# Created:     01-06-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------

from __future__ import division
import numpy as np
import cv2
import math, time
from Error import *
from picamera.array import PiRGBArray
from picamera import PiCamera
#import ArduinoComm
DEBUG = True
font = cv2.FONT_HERSHEY_SIMPLEX

c_frameWidth = 640
c_frameHeight = 480
c_gaussianBlurTuple = (5,5)

#video
c_redLower1 = np.array([0,100,100])
c_redUpper1 = np.array([10,255,255])
c_redLower2 = np.array([170,50,50])
c_redUpper2 = np.array([180,255,255])
c_blueLower_t = np.array([90,100,100])
c_blueUpper_t = np.array([130,255,255])
c_blueLower = np.array([90,100,0])
c_blueUpper = np.array([140,255,110])
c_yellowLower = np.array([14,108,173])
c_yellowUpper = np.array([21,255,255])
c_whiteLower = np.array([0,0,210])
c_whiteUpper = np.array([170,25,255])


c_signWhiteRatio = 0.55

c_redAreaMin = 500
c_blueAreaMin = 400
c_yellowAreaMin = 500

c_toleranceOctagon = 0.03
c_toleranceDiamond = 0.01

c_roiRescaleAdder = 10

c_circleThresh1 = 1.7
c_circleThresh2 = 70

c_circleDetectionLower = 0.75
c_circleDetectionUpper = 1.15

c_signAreaClassifierThresh = 0.3

c_disEstHeight = 300
c_disEstWidth  = 300


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(0.1)

class TrafficSign():
    def __init__(self):
        self.__InitSuccess = True
        #self.__comm = ard_comm
        self.__GetTemplates()

    def __GetTemplates(self):
        self.__l_temp = cv2.imread("templates/l_template.jpg")
        self.__l_temp = cv2.cvtColor(self.__l_temp, cv2.COLOR_BGR2HSV)
        self.__r_temp = cv2.imread("templates/r_template.jpg")
        self.__r_temp = cv2.cvtColor(self.__r_temp, cv2.COLOR_BGR2HSV)
        self.__s_temp = cv2.imread("templates/s_template.jpg")
        self.__s_temp = cv2.cvtColor(self.__s_temp, cv2.COLOR_BGR2HSV)

    def __SplitColors(self):
        self.__img = cv2.medianBlur(self.__imgOrig,3)
        #self.__kernel = np.ones((3,3),np.uint8)
        self.__hsv = cv2.cvtColor(self.__img, cv2.COLOR_BGR2HSV)
	mask1 = cv2.inRange(self.__hsv, c_redLower1,c_redUpper1)
	mask2 = cv2.inRange(self.__hsv, c_redLower2,c_redUpper2)
        self.__red = cv2.bitwise_or(mask1,mask2)
        #self.__red = cv2.erode(self.__red, self.__kernel)
        self.__blue = cv2.inRange(self.__hsv, c_blueLower, c_blueUpper)
        #self.__blue = cv2.erode(self.__blue, self.__kernel)
        self.__yellow = cv2.inRange(self.__hsv, c_yellowLower, c_yellowUpper)
        #self.__yellow = cv2.erode(self.__yellow, self.__kernel)
        cv2.imshow("red",self.__red)
        cv2.imshow("blue",self.__blue)
        cv2.imshow("yellow",self.__yellow)

    def __IdentifyContours(self):
        self.__red_c, self.__hier_r = cv2.findContours(self.__red,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        self.__blue_c, self.__hier_b = cv2.findContours(self.__blue,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        self.__yellow_c, self.__hier_y = cv2.findContours(self.__yellow,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

    def __FindStopSign(self):
        if self.__red_c is not None:
            iterator = 0
            for cnt in self.__red_c:
                area = cv2.contourArea(cnt)
                if area>c_redAreaMin and self.__hier_r[0][iterator][3]==-1:
                    ep_r = c_toleranceOctagon*cv2.arcLength(cnt,True)
                    app_r = cv2.approxPolyDP(cnt,ep_r,True)
                    p = cv2.arcLength(cnt,True)
                    if len(app_r)==8 and self.__hier_r[0][iterator][2]!=-1:
                        x,y,w,h = cv2.boundingRect(cnt)
                        cv2.rectangle(self.__img, (x,y), (x+w,y+h), (0,255,0),2)
                        cv2.putText(self.__img,'stop',(x,y), font, 1,(255,255,255),2)
                        self.__EstimateDistanceToSign(x,y,w,h)
                        #self.__comm.SendMessage("STOP",[0,0,0,0])
                iterator=iterator+1

    def __FindUTurnSign(self):
        if self.__yellow_c is not None:
            for cnt in self.__yellow_c:
                area = cv2.contourArea(cnt)
                if area > c_yellowAreaMin:
                    ep_y = c_toleranceDiamond*cv2.arcLength(cnt,True)
                    app_y = cv2.approxPolyDP(cnt,ep_y,True)
                    if len(app_y)==4:
                        pts = app_y
                        y=cnt
                        x,y,w,h = cv2.boundingRect(cnt)
                        cv2.rectangle(self.__img,(x,y),(x+w,y+h),(0,255,0),2)
                        cv2.putText(self.__img,'u-turn',(x,y), font, 1,(255,255,255),2)
                        self.__EstimateDistanceToSign(x,y,w,h)


    def __SecondaryTurnTest(self, x, y, w, h):
        xx = x - c_roiRescaleAdder
        yy = y - c_roiRescaleAdder
        ww = w + (2*c_roiRescaleAdder)
        hh = h + (2*c_roiRescaleAdder)
        roi = self.__img[yy:yy+hh,xx:xx+ww]
        gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
        try:
            circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, c_circleThresh1 , c_circleThresh2)
        except:
            circles = None
        if circles is not None:
            circles = np.round(circles[0,:]).astype("int")
            return True
        else:
            return False

    def __EstimateDistanceToSign(self,x,y,w,h):
        approx_distance = (c_disEstHeight*c_disEstWidth)/(w*h)
        dist = "d="+str(("%.2f" % approx_distance))
        cv2.putText(self.__img,dist,(x,y+h), font, 1,(255,255,255),2)


    def __BlueKernel(self, x, y, w, h):
        roi = self.__img[y:y+h,x:x+w]
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        temp_blue = cv2.inRange(roi,c_blueLower,c_blueUpper)
        white_pix = [None,None,None]
        temp_l = cv2.resize(self.__l_temp,(w,h),interpolation = cv2.INTER_LINEAR)
        temp_l = cv2.inRange(temp_l,c_blueLower_t,c_blueUpper_t)
        temp_r = cv2.resize(self.__r_temp,(w,h),interpolation = cv2.INTER_LINEAR)
        temp_r = cv2.inRange(temp_r,c_blueLower_t,c_blueUpper_t)
        temp_s = cv2.resize(self.__s_temp,(w,h),interpolation = cv2.INTER_LINEAR)
        temp_s = cv2.inRange(temp_s,c_blueLower_t,c_blueUpper_t)
        l_and = cv2.bitwise_and(temp_blue,temp_l)
        r_and = cv2.bitwise_and(temp_blue,temp_r)
        s_and = cv2.bitwise_and(temp_blue,temp_s)
        white_pix[0] = cv2.findNonZero(l_and)
        white_pix[1] = cv2.findNonZero(r_and)
        white_pix[2] = cv2.findNonZero(s_and)
        for i in range(0,3):
            if white_pix[i] == None:
                white_pix[i] = 0
            else:
                white_pix[i] = white_pix[i].size
        cv2.rectangle(self.__img,(x,y),(x+w,y+h),(0,255,0),2)
        self.__EstimateDistanceToSign(x,y,w,h)
        if white_pix[0]>white_pix[1] and white_pix[0]>white_pix[2]:
            cv2.putText(self.__img,'left',(x,y), font, 1,(255,255,255),2)
            #self.__comm.SendMessage("ARC_TURN",[0,210,1,0])
        elif white_pix[1]>white_pix[0] and white_pix[1]>white_pix[2]:
            cv2.putText(self.__img,'right',(x,y), font, 1,(255,255,255),2)
            #self.__comm.SendMessage("ARC_TURN",[210,0,1,0])
        elif white_pix[2]>white_pix[0] and white_pix[2]>white_pix[1]:
            cv2.putText(self.__img,'straight',(x,y), font, 1,(255,255,255),2)
            #self.__comm.SendMessage("FORWARD",[210,100,1,0])
##        cv2.imshow("temp_blue",temp_blue)
##        cv2.imshow("temp_l",l_and)
##        cv2.imshow("temp_r",r_and)
##        cv2.imshow("temp_s",s_and)


    def __FindTurnSign(self):
        if self.__blue_c is not None:
            for cnt in self.__blue_c:
                area = cv2.contourArea(cnt)
                if area > c_blueAreaMin:
                    p = cv2.arcLength(cnt,True)
                    x,y,w,h = cv2.boundingRect(cnt)
                    if p!=0:
                        T = 4*np.pi*(area/(p*p))
                        if T > c_circleDetectionLower and T < c_circleDetectionUpper:
                            #print "test1 passed"
                            self.__BlueKernel(x,y,w,h)
                        else:
                            if (self.__SecondaryTurnTest(x,y,w,h,)):
                                self.__BlueKernel(x,y,w,h)

    def DetectSign(self, img_orig):
        self.__imgOrig = img_orig
        self.__SplitColors()
        self.__IdentifyContours()
        self.__FindStopSign()
        self.__FindUTurnSign()
        self.__FindTurnSign()
        cv2.imshow("img",self.__img)

if __name__ == '__main__':

    #comm = ArduinoComm.ArduinoComm("/dev/ttyUSB0",9600)
    signObj = TrafficSign()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #time.sleep(0.05)
        img_orig = frame.array
        signObj.DetectSign(img_orig)
	
	key = cv2.waitKey(1) & 0xFF
	 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
	    break
