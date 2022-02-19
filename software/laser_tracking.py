from os import kill
import cv2
import numpy as np
from numpy.core.defchararray import greater
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
PARAMETERS = cv2.aruco.DetectorParameters_create()
RESOLUTION = (400, 400)
FRAMERATE = 60

error_x = None
error_y = None
kill_loop = False

def find_markers(frame):
    global error_x, error_y
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, DICTIONARY, parameters=PARAMETERS)
    if ids is not None and 0 in ids:
        for corner, id in zip(corners, ids):
            if id == 0:
                error_x = RESOLUTION[0]/2 - np.average(corner[0][:, 0])
                error_y = RESOLUTION[0]/2 - np.average(corner[0][:, 1])
    else:
        error_x = None
        error_y = None


def run_tracking_loop(debug=False):
    global kill_loop
    kill_loop = False
    with PiCamera() as camera:
        camera.resolution = RESOLUTION
        camera.framerate = FRAMERATE
        camera.shutter_speed = 200
        raw_capture = PiRGBArray(camera, size=RESOLUTION)
        for raw in camera.capture_continuous(raw_capture, format='bgr', use_video_port=True):
            image = raw.array.copy()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([155, 90, 100])
            lower_red = np.array([179, 255, 255])
            lower = np.array([120, 0, 100])
            upper = np.array([200, 255, 255])
            mask = cv2.inRange(image, lower, upper)
            red_mask = cv2.inRange(image, lower_red, upper)
            M = cv2.moments(mask)
            # calculate x,y coordinate of center
            if M["m00"] != 0:
                laser_x = int(M["m01"] / M["m00"])
            else:
                laser_x = 400
            print(laser_x)

            # cv2.circle(raw.array, (cX, cY), 5, (255, 255, 255), -1)
            # cv2.putText(raw.array, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # cv2.imshow('mask', mask)
            # cv2.imshow('mask', red_mask)
            # cv2.imshow('result', raw.array)
            # cv2.waitKey(10)
            raw_capture.truncate(0)

def end_tracking_loop():
    global kill_loop
    kill_loop = True

def get_error():
    return error_x, error_y

if __name__ == '__main__':    
    run_tracking_loop(debug=True)
