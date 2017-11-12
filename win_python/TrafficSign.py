#-------------------------------------------------------------------------------
# Name:        TrafficSignV3.py
# Purpose:     Traffic sign detection module
#
# Author:      Sri Muthu Narayanan Balasubramanian
#
# Created:     17-05-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------

from __future__ import division
import numpy as np
import cv2
import math, time
from Error import *
from LaneDetect import LaneDetect

DEBUG = False
VISUALIZATION = True
font = cv2.FONT_HERSHEY_SIMPLEX

c_frameWidth = 800
c_frameHeight = 600
c_gaussianBlurTuple = (3,3)

c_RoiRatio = 0.4

c_leftTemplate = "templates/l_template.jpg"
c_rightTemplate = "templates/r_template.jpg"
c_straightTemplate = "templates/s_template.jpg"

#video
c_redLower1 = np.array([170,50,50])
c_redUpper1 = np.array([180,255,255])
c_redLower2 = np.array([0,100,100])
c_redUpper2 = np.array([15,255,255])
c_blueLower = np.array([90,100,40])
c_blueUpper = np.array([130,255,255])
c_blueLow = np.array([90,100,40])
c_blueUp = np.array([130,255,180])
c_yellowLower = np.array([10,150,50])
c_yellowUpper = np.array([30,255,255])
c_whiteLower = np.array([0,0,210])
c_whiteUpper = np.array([170,25,255])

c_signConfidenceInv = 0.4

c_redAreaMin = 1000
c_blueAreaMin = 1000
c_yellowAreaMin = 1000

c_toleranceOctagon = 0.03
c_toleranceDiamond = 0.01

c_roiRescaleAdder = 10

c_circleThresh1 = 1.7
c_circleThresh2 = 70

c_circleDetectionLower = 0.75
c_circleDetectionUpper = 1.15

c_signAreaClassifierThresh = 0.3

#c_knownWidth = 18.5
#c_focalLength  = 797.838
c_knownWidth = 14.8
c_focalLength  = 782.43


class TrafficSign():
    def __init__(self):
        self.__InitSuccess = True
        self.__GetTemplates()

    def __GetTemplates(self):
        self.__l_temp = cv2.imread(c_leftTemplate)
        self.__l_temp = cv2.cvtColor(self.__l_temp, cv2.COLOR_BGR2HSV)
        self.__r_temp = cv2.imread(c_rightTemplate)
        self.__r_temp = cv2.cvtColor(self.__r_temp, cv2.COLOR_BGR2HSV)
        self.__s_temp = cv2.imread(c_straightTemplate)
        self.__s_temp = cv2.cvtColor(self.__s_temp, cv2.COLOR_BGR2HSV)

    def __RescaleImage(self):
        self.__imgOrig = cv2.resize(self.__imgOrig,(c_frameWidth,c_frameHeight),interpolation = cv2.INTER_LINEAR)
        self.__size_x = self.__imgOrig.shape[1]
        self.__size_y = self.__imgOrig.shape[0]
        self.__ROI_w = self.__size_x
        self.__ROI_h = self.__size_y * c_RoiRatio
        self.__ROI_x = 0
        self.__ROI_y = 0

    def __ReInitRetVar(self):
        self.__signPerIter = {}

    def __CropRoi(self):

        #self.__imgGray = cv2.cvtColor(self.__imgOrig,cv2.COLOR_BGR2GRAY)
        #self.__imgHist = cv2.equalizeHist(self.__imgGray)
        self.__imgOrig = self.__imgOrig[int(self.__ROI_y):int(self.__ROI_y)+int(self.__ROI_h),
                                        int(self.__ROI_x):int(self.__ROI_x)+int(self.__ROI_w)]

        self.__img = cv2.GaussianBlur(self.__imgOrig,c_gaussianBlurTuple,0)

    def __SplitColors(self):
#        self.__img = cv2.medianBlur(self.__imgOrig,3)
#        self.__img = cv2.GaussianBlur(self.__imgOrig,(5,5),0)
        self.__kernel = np.ones((3,3),np.uint8)
        self.__kernel1 = np.ones((1,1),np.uint8)

        self.__hsv = cv2.cvtColor(self.__img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(self.__hsv,c_redLower1,c_redUpper1)
        mask2 = cv2.inRange(self.__hsv,c_redLower2,c_redUpper2)
        self.__red = cv2.bitwise_or(mask1,mask2)
        self.__red = cv2.dilate(self.__red, self.__kernel1)

        self.__blue = cv2.inRange(self.__hsv, c_blueLower, c_blueUpper)
        self.__blue = cv2.dilate(self.__blue, self.__kernel)

        self.__yellow = cv2.inRange(self.__hsv, c_yellowLower, c_yellowUpper)
        self.__yellow = cv2.erode(self.__yellow, self.__kernel)

        if DEBUG:
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
                        if p!=0:
                            T = 4*np.pi*(area/(p*p))
                            if T>c_circleDetectionLower and T<c_circleDetectionUpper:
                                x,y,w,h = cv2.boundingRect(cnt)
                                distance = self.__EstimateDistanceToSign(x, y, w, h)

                                if VISUALIZATION:
                                    cv2.rectangle(self.__img, (x,y), (x+w,y+h), (0,255,0),2)
                                    cv2.putText(self.__img,'stop',(x,y), font, 1,(255,255,255),2)
                                self.__signPerIter["STOP"] = distance
                iterator=iterator+1

    def __FindUTurnSign(self):
        tempUTurns = []
        if self.__yellow_c is not None:
            for cnt in self.__yellow_c:
                area = cv2.contourArea(cnt)
                if area > c_yellowAreaMin:
                    ep_y = c_toleranceDiamond*cv2.arcLength(cnt,True)
                    app_y = cv2.approxPolyDP(cnt,ep_y,True)
                    if len(app_y)>=4 or len(app_y)<=5:
                        pts = app_y
                        y=cnt
                        x,y,w,h = cv2.boundingRect(cnt)
                        distance = self.__EstimateDistanceToSign(x, y, w, h)
                        tempUTurns.append(distance)
                        if VISUALIZATION:
                            cv2.rectangle(self.__img,(x,y),(x+w,y+h),(0,255,0),2)
                            cv2.putText(self.__img,'u-turn',(x,y), font, 1,(255,255,255),2)
        if len(tempUTurns)==1:
            self.__signPerIter["UTURN"] = distance






    def __SecondaryTurnTest(self, x, y, w, h):
        xx = x - c_roiRescaleAdder
        yy = y - c_roiRescaleAdder
        ww = w + (2*c_roiRescaleAdder)
        hh = h + (2*c_roiRescaleAdder)
        roi = self.__img[yy:yy+hh,xx:xx+ww]
        gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
        try:
            roi = self.__img[yy:yy+hh,xx:xx+ww]
            gray = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
            if gray is not None:
                circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, c_circleThresh1 , c_circleThresh2)
            else:
                circles = None
        except:
            circles = None
        if circles is not None:
            for circle in circles:
                if (np.pi*circle[0][2]*circle[0][2])>(0.7*w*h):
                    if DEBUG:
                        print circles
                    if VISUALIZATION:
                        circles = np.round(circles[0,:]).astype("int")
                    return True
            return False
        else:
            return False

    def __EstimateDistanceToSign(self,x,y,w,h):
        approx_distance = (c_knownWidth*c_focalLength)/w
        dist = "d="+str(("%.2f" % approx_distance))+" cm"
        if VISUALIZATION:
            cv2.putText(self.__img,dist,(x,y+h), font, 1,(255,255,255),2)
        return int(approx_distance)

    def __BlueKernel(self, x, y, w, h):

        roi = self.__img[y:y+h,x:x+w]
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        temp_blue = cv2.inRange(roi,c_blueLow,c_blueUp)
        white_pix = [None,None,None]
        temp_l = cv2.resize(self.__l_temp,(w,h),interpolation = cv2.INTER_LINEAR)
        temp_l = cv2.inRange(temp_l,c_blueLower,c_blueUpper)
        temp_r = cv2.resize(self.__r_temp,(w,h),interpolation = cv2.INTER_LINEAR)
        temp_r = cv2.inRange(temp_r,c_blueLower,c_blueUpper)
        temp_s = cv2.resize(self.__s_temp,(w,h),interpolation = cv2.INTER_LINEAR)
        temp_s = cv2.inRange(temp_s,c_blueLower,c_blueUpper)
        # l_and = cv2.bitwise_and(temp_blue,temp_l)
        # r_and = cv2.bitwise_and(temp_blue,temp_r)
        # s_and = cv2.bitwise_and(temp_blue,temp_s)

        l_and = temp_l - temp_blue
        r_and = temp_r - temp_blue
        s_and = temp_s - temp_blue

        white_pix[0] = cv2.findNonZero(l_and)
        white_pix[1] = cv2.findNonZero(r_and)
        white_pix[2] = cv2.findNonZero(s_and)
        for i in range(0,3):
            if white_pix[i] is None:
                white_pix[i] = w*h
            else:
                white_pix[i] = white_pix[i].size

        confidenceInv = c_signConfidenceInv * w * h

        cv2.rectangle(self.__img,(x,y),(x+w,y+h),(0,255,0),2)
        distance = self.__EstimateDistanceToSign(x,y,w,h)
        if white_pix[0]<white_pix[1] and white_pix[0]<white_pix[2]:
            if white_pix[0]<confidenceInv:
                if VISUALIZATION:
                    cv2.putText(self.__img,'left',(x,y), font, 1,(255,255,255),2)
                self.__signPerIter["LEFT"] = distance
        elif white_pix[1]<white_pix[0] and white_pix[1]<white_pix[2]:
            if white_pix[1] < confidenceInv:
                if VISUALIZATION:
                    cv2.putText(self.__img,'right',(x,y), font, 1,(255,255,255),2)
                self.__signPerIter["RIGHT"] = distance
        elif white_pix[2]<white_pix[0] and white_pix[2]<white_pix[1]:
            if white_pix[1] < confidenceInv:
                if VISUALIZATION:
                    cv2.putText(self.__img,'straight',(x,y), font, 1,(255,255,255),2)
                #self.__signPerIter["STRAIGHT"] = distance

        if DEBUG:
            cv2.imshow("temp_blue",temp_blue)
            cv2.imshow("temp_l",l_and)
            cv2.imshow("temp_r",r_and)
            cv2.imshow("temp_s",s_and)


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
                            self.__BlueKernel(x,y,w,h)
                        else:
                            if (self.__SecondaryTurnTest(x,y,w,h,)):
                                self.__BlueKernel(x,y,w,h)

    def DetectSign(self, img_orig):
        self.__imgOrig = img_orig
        self.__RescaleImage()
        self.__CropRoi()
        self.__SplitColors()
        self.__IdentifyContours()
        self.__ReInitRetVar()
        self.__FindStopSign()
        self.__FindUTurnSign()
        self.__FindTurnSign()
        if VISUALIZATION:
            cv2.imshow("img",self.__img)
        return self.__signPerIter

if __name__ == '__main__':

    cap = cv2.VideoCapture("sat_2_93d.h264")
    print "video capture opening :",cap.isOpened()
    signObj = TrafficSign()
    laneObj = LaneDetect()
    while(cap.isOpened()):
        #time.sleep(0.05)
        ret, img_orig = cap.read()
        signObj.DetectSign(img_orig)
        laneObj.DetectLane(img_orig)
        if DEBUG:
            cv2.waitKey(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()