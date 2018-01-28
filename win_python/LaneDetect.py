# -------------------------------------------------------------------------------
# Name:        LaneDetect.py
# Purpose:     Lane detection module
#
# Author:      Sri Muthu Narayanan Balasubramanian
#
# Created:     23-11-2017
# Copyright:   (c) User 2016
# -------------------------------------------------------------------------------
"""
This file contains the LaneDetect class used to
detect a lane in a video/image and returns
1. Area of the free unencumbered space
2. Estimated length of free path ahead (in pixels)
3. Angle correction required from the center line (set point)
"""
# System imports
from __future__ import division
import math
import time
import sys

# Module specific imports
import cv2
import numpy as np

# Runtime behavior flags
VISUALIZATION = True
DEBUG = True

# Set to enable adaptive threshold
ADAPTIVE = False

# Macros
IMAGE_X_INDEX = 1
IMAGE_Y_INDEX = 0

# Constants
FONT = cv2.FONT_HERSHEY_SIMPLEX

FRAME_WIDTH = 800
FRAME_HEIGHT = 600
ROI_FRAME_RATIO = 0.7
ROWS = int(FRAME_HEIGHT * ROI_FRAME_RATIO)
COLS = int(FRAME_WIDTH)

FRAMES = 60

GAUSSIAN_BLUR_TUPLE = (5, 5)
MEDIAN_BLUR_INDEX = 3
ADAPTIVE1 = 15
ADAPTIVE2 = 2
BW_THRESH = 85
EROSION_ITERATIONS = 2

MIN_CONTOUR_AREA = 10000

# Sensitivity of set point (smaller number is higher sensitivity)
SENSITIVITY = 0.4


def rescale_image(image):
    """
    Rescales image to pre-determined dimensions
    :param image: input image
    :return: rescaled_image
    """
    rescaled_image = cv2.resize(image, (FRAME_WIDTH, FRAME_HEIGHT),
                                interpolation=cv2.INTER_LINEAR)
    return rescaled_image


def construct_pathfinder_polygon():
    """
    Return the points for the pathfinder polygon
    :return:  points for the pathfinder polygon
    """
    poly_points = np.array([[int(COLS * 0.25), ROWS],
                            [int(COLS * 0.75), ROWS],
                            [COLS, int(ROWS * SENSITIVITY)],
                            [COLS, int(ROWS * 0.25)],
                            [0, int(ROWS * 0.25)],
                            [0, int(ROWS * SENSITIVITY)]], np.int32)

    return poly_points


class LaneDetect(object):
    """
    LaneDetection class definition
    """
    def __init__(self):
        self.__fps_params = {'frameCount': 0, 'start': 0, 'end': 0, 'fps': 0}
        self.rescaled_frame = None
        self.cropped_color_frame = None
        self.cropped_frame = None
        self.pathfinder_polygon = construct_pathfinder_polygon()

    def estimate_fps(self):
        """
        Estimate the frames per second processed
        :return: Nothing (show fps on the image)
        """
        if self.__fps_params['frameCount'] >= FRAMES:
            self.__fps_params['end'] = time.time()
            exec_time = self.__fps_params['end'] - self.__fps_params['start']
            self.__fps_params['fps'] = FRAMES / exec_time
            self.__fps_params['start'] = time.time()
            self.__fps_params['frameCount'] = 0
        self.__fps_params['frameCount'] += 1

        cv2.putText(self.cropped_color_frame, str(int(self.__fps_params['fps'])),
                    (30, 30), FONT, 1, (255, 0, 0), 2)

    def crop_roi_and_convert2gray(self, image):
        """
        Crop Region of Interest based on the ROI_FRAME_RATIO
        Convert ot gray scale and Compute the roi_vector
        :param image: input image
        :return: Cropped image (gray scale)
        """
        roi_vector = {'x': 0, 'y': 0, 'w': 0, 'h': 0}
        roi_vector['w'] = image.shape[IMAGE_X_INDEX]
        roi_vector['h'] = int(image.shape[IMAGE_Y_INDEX] * ROI_FRAME_RATIO)
        roi_vector['x'] = 0
        roi_vector['y'] = image.shape[IMAGE_Y_INDEX] - roi_vector['h']
        y_start = roi_vector['y']
        y_end = roi_vector['y'] + roi_vector['h']
        x_start = roi_vector['x']
        x_end = roi_vector['x'] + roi_vector['w']
        self.cropped_color_frame = image[y_start:y_end, x_start:x_end]
        self.cropped_frame = cv2.cvtColor(self.cropped_color_frame, cv2.COLOR_BGR2GRAY)
        self.cropped_frame = cv2.medianBlur(self.cropped_frame, MEDIAN_BLUR_INDEX)
        self.cropped_frame = cv2.GaussianBlur(self.cropped_frame, GAUSSIAN_BLUR_TUPLE, 0)

    def superimpose_pathfinder_polygon(self):
        """
        Superimpose the pathfinder polygon on the cropped frame
        :return: Superimposes polygon on self.cropped_frame
        """
        cv2.polylines(self.cropped_frame, [self.pathfinder_polygon],
                      True, (0, 0, 0), 3)
        if VISUALIZATION:
            cv2.line(self.cropped_color_frame,
                     (int(COLS / 2), ROWS), (int(COLS / 2), 0),
                     (255, 0, 0), 1)

    def bin_and_find_contours(self):
        """
        Binarize the gray scale image and find the contours
        :return: contours, hierarchy
        """
        if not ADAPTIVE:
            ret, bin_image = cv2.threshold(self.cropped_frame, BW_THRESH,
                                           255, cv2.THRESH_BINARY)
        else:
            bin_image = cv2.adaptiveThreshold(self.cropped_frame, 255,
                                              cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                              cv2.THRESH_BINARY,
                                              ADAPTIVE1, ADAPTIVE2)
        kernel = np.ones((3, 3), np.uint8)
        bin_image = cv2.erode(bin_image, kernel,
                              iterations=EROSION_ITERATIONS)

        if DEBUG:
            cv2.imshow("binary image", bin_image)

        contours, hierarchy = cv2.findContours(bin_image, cv2.RETR_LIST,
                                               cv2.CHAIN_APPROX_SIMPLE)
        return contours, hierarchy

    def find_unobstructed_path(self, contours):
        """
        Find the unobstructed path
        :param contours: contours from bin image
        :return:    angle of deviation from center line
                    unobstructed distance in pixels
                    unobstructed area in pixels
        """

        angle_deviation = 0
        distance_unobstructed = 0
        area_unobstructed = 0

        rows, cols = self.cropped_frame.shape[:2]
        if contours is None:
            return -1, -1, -1

        y_list = []
        cnt_list = []
        for cnt in contours:
            if cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
                moments = cv2.moments(cnt)
                com_x = int(moments['m10'] / moments['m00'])
                com_y = int(moments['m01'] / moments['m00'])
                if cv2.pointPolygonTest(self.pathfinder_polygon,
                                        (com_x, com_y), True) > 0:
                    cnt_list.append([cnt, com_x, com_y])
                    y_list.append(com_y)

        if y_list is not None:
            y_list.sort()
        else:
            return -1, -1, -1

        for item in cnt_list:
            if item[2] == y_list[-1]:
                angle_deviation = int(math.atan((int(cols / 2) - item[1])
                                                / (rows - item[2])) * 180 / math.pi)
                distance_unobstructed = int(math.sqrt(math.pow((int(cols / 2) - item[1]), 2)
                                                      + math.pow((rows - item[2]), 2)))
                area_unobstructed = int(cv2.contourArea(item[0]))

                if VISUALIZATION:
                    cv2.circle(self.cropped_color_frame, (item[1], item[2]), 3, 3)
                    cv2.drawContours(self.cropped_color_frame, [item[0]], 0, (0, 255, 0), 2)
                    cv2.line(self.cropped_color_frame, (int(cols / 2), rows),
                             (item[1], item[2]), (0, 255, 0), 1)
                    cv2.putText(self.cropped_color_frame, str(angle_deviation),
                                (item[1] + 10, item[2] + 10), FONT, 1, (255, 0, 0), 2)
                    cv2.putText(self.cropped_color_frame, str(distance_unobstructed),
                                (item[1] - 20, item[2] - 20), FONT, 1, (255, 0, 0), 2)

        return angle_deviation, distance_unobstructed, area_unobstructed

    def detect_lane(self, image):
        """
        The main function for lane detection
        :param image: input image (frame)
        :return:
        """
        self.rescaled_frame = rescale_image(image)
        self.crop_roi_and_convert2gray(self.rescaled_frame)
        self.superimpose_pathfinder_polygon()
        contours, hierarchy = self.bin_and_find_contours()
        self.find_unobstructed_path(contours)

        if VISUALIZATION:
            self.estimate_fps()
            cv2.imshow("Cropped_color_frame", self.cropped_color_frame)
        if DEBUG:
            cv2.imshow("Internal", self.cropped_frame)

if __name__ == '__main__':  # For testing purposes

    CAP = cv2.VideoCapture("sat_2_93d.h264")
    if CAP.isOpened():
        print "Video capture successfully opened"
    else:
        sys.exit("Capture open failed! Exiting")
    LANE_OBJ = LaneDetect()
    while CAP.isOpened():
        RET, FRAME = CAP.read()
        LANE_OBJ.detect_lane(FRAME)
        if DEBUG:
            cv2.waitKey(0)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    CAP.release()
    cv2.destroyAllWindows()
