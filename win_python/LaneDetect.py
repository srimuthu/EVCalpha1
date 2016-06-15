#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      User
#
# Created:     09-06-2016
# Copyright:   (c) User 2016
# Licence:     <your licence>
#-------------------------------------------------------------------------------

from __future__ import division
import numpy as np
import cv2
import math, time
from Error import *

font = cv2.FONT_HERSHEY_SIMPLEX

c_frameWidth = 640
c_frameHeight = 480
c_gaussianBlurTuple = (3,3)

CANNY_MIN_TRESHOLD = 300
CANNY_MAX_TRESHOLD = 400

class LaneDetect():
    def __init__(self):
        self.__init____InitSuccess = True
##        self.__size_x = c_frameWidth
##        self.__size_y = c_frameHeight
##        self.__ROI_w = self.__size_x
##        self.__ROI_h = self.__size_y/2
##        self.__ROI_x = 0
##        self.__ROI_y = self.__size_y - self.__ROI_h

    def __RescaleImage(self):
        self.__imgOrig = cv2.resize(self.__imgOrig,(c_frameWidth,c_frameHeight),interpolation = cv2.INTER_LINEAR)
        self.__size_x = self.__imgOrig.shape[1]
        self.__size_y = self.__imgOrig.shape[0]
        self.__ROI_w = self.__size_x
        self.__ROI_h = self.__size_y/2
        self.__ROI_x = 0
        self.__ROI_y = self.__size_y - self.__ROI_h

    def __CropRoi(self):

        self.__imgGray = cv2.cvtColor(self.__imgOrig,cv2.COLOR_BGR2GRAY)
        self.__imgHist = cv2.equalizeHist(self.__imgGray)
        self.__imgRoi = self.__imgHist[self.__ROI_y:self.__ROI_y+self.__ROI_h,
                                        self.__ROI_x:self.__ROI_x+self.__ROI_w]

        self.__imgRoi = cv2.GaussianBlur(self.__imgRoi,c_gaussianBlurTuple,0)

    def __DetectLines(self):

        self.__imgEdge = cv2.Canny(self.__imgRoi,CANNY_MIN_TRESHOLD,CANNY_MAX_TRESHOLD)
        self.__lines = cv2.HoughLines(self.__imgEdge,1,np.pi/180,100)

    def __PrintLines(self):
        try:
            for rho,theta in self.__lines[0]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                print theta

            cv2.line(self.__imgRoi,(x1,y1),(x2,y2),(0,0,255),2)
        except:
            pass

    def DetectLane(self, img_orig):
        self.__imgOrig = img_orig
        self.__RescaleImage()
        self.__CropRoi()
        self.__DetectLines()
        #self.__FindLanes()
        self.__PrintLines()
        cv2.imshow("img",self.__imgRoi)

if __name__ == '__main__':

    cap = cv2.VideoCapture("video_new.h264")
    print "video capture opening :",cap.isOpened()
    laneObj = LaneDetect()
    while(cap.isOpened()):
        time.sleep(0.05)
        ret, img_orig = cap.read()
        laneObj.DetectLane(img_orig)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()