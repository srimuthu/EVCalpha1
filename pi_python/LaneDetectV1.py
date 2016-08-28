# -------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      User
#
# Created:     09-06-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
# -------------------------------------------------------------------------------

from __future__ import division
import numpy as np
import cv2
import math, time
from Error import *
from picamera.array import PiRGBArray
from picamera import PiCamera

c_frames = 60
c_roiFrameRatio = 0.75
c_bwThresh = 75
c_minContourArea = 3000
c_erosionIterations = 1
##Sensitivity of setpoint (smaller number is higher sensitivity)
c_sensitivity = 0.4

font = cv2.FONT_HERSHEY_SIMPLEX

c_frameWidth = 800
c_frameHeight = 600
c_gaussianBlurTuple = (3, 3)


class LaneDetect():
    def __init__(self):
        self.__init____InitSuccess = True
        self.__frameCount = 0
        self.__fps = 0
        self.__polyCx = 0
        self.__polyCy = 0
        self.__cntList = []
        self.__yList = []
        self.__start = time.time()

    def __RescaleImage(self):
        self.__imgOrig = cv2.resize(self.__imgOrig, (c_frameWidth, c_frameHeight),
                                    interpolation=cv2.INTER_LINEAR)
        self.__size_x = self.__imgOrig.shape[1]
        self.__size_y = self.__imgOrig.shape[0]
        self.__ROI_w = self.__size_x
        self.__ROI_h = int(self.__size_y * c_roiFrameRatio)
        self.__ROI_x = 0
        self.__ROI_y = self.__size_y - self.__ROI_h

    def __CropRoi(self):

        self.__imgRoiColor = self.__imgOrig[self.__ROI_y:self.__ROI_y + self.__ROI_h,
                             self.__ROI_x:self.__ROI_x + self.__ROI_w]
        self.__imgGray = cv2.cvtColor(self.__imgOrig, cv2.COLOR_BGR2GRAY)
        # self.__imgGray = cv2.equalizeHist(self.__imgGray)
        self.__imgRoi = self.__imgGray[self.__ROI_y:self.__ROI_y + self.__ROI_h,
                        self.__ROI_x:self.__ROI_x + self.__ROI_w]
        self.__imgRoi = cv2.medianBlur(self.__imgRoi, 3)
        self.__imgRoi = cv2.GaussianBlur(self.__imgRoi, c_gaussianBlurTuple, 0)

    def __RobotPath(self):
        rows, cols = self.__imgRoi.shape[:2]
        ##Primary Shape
        # self.__polyPoints = np.array([[0,rows],[cols,rows],[cols,int(rows*0.5)],
        #                         [int(cols*0.75),int(rows*0.25)],[int(cols*0.25),
        #                             int(rows*0.25)],[0,int(rows*0.5)],[0,int(rows/2)]], np.int32)

        ##Diamond Shape
        # self.__polyPoints = np.array([[int(cols*0.25),rows],[int(cols*0.75),rows],[cols,int(rows*0.5)],
        #                         [int(cols*0.75),int(rows*0.25)],[int(cols*0.25),
        #                             int(rows*0.25)],[0,int(rows*0.5)]], np.int32)

        ##Inverted Primary shape
        self.__polyPoints = np.array([[int(cols * 0.25), rows],
                                      [int(cols * 0.75), rows],
                                      [cols, int(rows * c_sensitivity)],
                                      [cols, int(rows * 0.25)],
                                      [0,int(rows * 0.25)],
                                      [0, int(rows * c_sensitivity)]], np.int32)

        ##Tuple cope of primary shape
        # self.__polyTuple = [(0,rows),(cols,rows),(cols,int(rows*0.5)),
        #                     (int(cols*0.75),int(rows*0.25)),(int(cols*0.25),
        #                             int(rows*0.25)),(0,int(rows*0.5)),(0,int(rows/2))]

        self.__polyCx = int(cols / 2)
        self.__polyCy = int(rows / 2)
        cv2.polylines(self.__imgRoi, [self.__polyPoints], True, (0, 0, 0), 3)
        cv2.line(self.__imgRoiColor, (int(cols / 2), rows), (int(cols / 2), 0), (255, 0, 0), 1)

    def __DetectPath(self):

        self.__RobotPath()

        rows, cols = self.__imgRoi.shape[:2]
        retTuple = ()

        ret, thresh1 = cv2.threshold(self.__imgRoi, c_bwThresh, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        thresh1 = cv2.erode(thresh1, kernel, iterations=c_erosionIterations)
        #cv2.imshow("bin", thresh1)
        contours, hier = cv2.findContours(thresh1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if contours is not None:
            for cnt in contours:
                if cv2.contourArea(cnt) > c_minContourArea:
                    M = cv2.moments(cnt)
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    if cv2.pointPolygonTest(self.__polyPoints, (cx, cy), True) > 0:
                        self.__cntList.append([cnt, cx, cy])
                        self.__yList.append(cy)
        if self.__yList is not None:
            self.__yList.sort()
            for item in self.__cntList:
                if item[2] == self.__yList[-1]:
                    cv2.circle(self.__imgRoiColor, (item[1], item[2]), 3, 3)
                    cv2.drawContours(self.__imgRoiColor, [item[0]], 0, (0, 255, 0), 2)
                    cv2.line(self.__imgRoiColor, (int(cols / 2), rows), (item[1], item[2]), (0, 255, 0), 1)

                    angle = int(math.atan((int(cols/2)-item[1])/(rows-item[2]))*180/math.pi)
                    length = int(math.sqrt(math.pow((int(cols/2)-item[1]),2)+math.pow((rows-item[2]),2)))
                    retTuple = (angle,length)
                    cv2.putText(self.__imgRoiColor, str(angle), (item[1]+10, item[2]+10), font, 1, (255, 0, 0), 2)
                    cv2.putText(self.__imgRoiColor, str(length), (item[1] - 20, item[2] - 20), font, 1, (255, 0, 0), 2)


        self.__yList = []
        self.__cntList = []
        return retTuple

    def __EstimateFPS(self):
        self.__frameCount = self.__frameCount + 1
        if self.__frameCount >= c_frames:
            self.__end = time.time()
            self.__time = self.__end - self.__start
            self.__fps = c_frames / self.__time
            self.__start = time.time()
            self.__frameCount = 0

        cv2.putText(self.__imgRoiColor, str(int(self.__fps)), (20, 20), font, 1, (255, 0, 0), 2)
        # cv2.circle(self.__imgRoiColor, (self.__polyCx, self.__polyCy), 3, 3)

    def DetectLane(self, img_orig):
        self.__imgOrig = img_orig
        self.__RescaleImage()
        self.__CropRoi()
        self.__EstimateFPS()
        self.__DetectPath()

        cv2.imshow("TSD", self.__imgRoiColor)

if __name__ == '__main__':

    #comm = ArduinoComm.ArduinoComm("/dev/ttyUSB0",9600)
    laneObj = LaneDetect()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        #time.sleep(0.05)
        img_orig = frame.array
        laneObj.DetectLane(img_orig)

	key = cv2.waitKey(1) & 0xFF

	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
	    break
		
