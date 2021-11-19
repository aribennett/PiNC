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
FRAMERATE = 30

error_x = None
error_y = None
laser_x = 0
kill_loop = False
enable_fiducial = False
enable_laser = False

def enable_laser_sensing():
    global enable_laser
    enable_laser = True


def enable_fiducial_sensing():
    global enable_fiducial
    enable_fiducial = True


def get_laser_displacement():
    return laser_x-RESOLUTION[0]/2

def find_markers(frame):
    global error_x,error_y
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


def find_laser(image):
    global laser_x
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([0,0,240])
    upper = np.array([255,255,255])
    mask = cv2.inRange(image, lower, upper)
    M = cv2.moments(mask)
    # calculate x coordinate of center
    if M["m00"] != 0:
        laser_x = int(M["m10"] / M["m00"])
    else:
        laser_x = 400

    lower_red = np.array([155,90,100])
    upper_red = np.array([179,255,255])
    mask2 = cv2.inRange(image, lower_red, upper_red)
    M2 = cv2.moments(mask2)
    # no laser
    if M2["m00"] == 0:
        laser_x = 400

def run_tracking_loop(debug=False):
    global kill_loop, enable_fiducial, enable_laser
    kill_loop = False
    with PiCamera() as camera:
        camera.resolution = RESOLUTION
        camera.framerate = FRAMERATE
        camera.shutter_speed = 2000
        raw_capture = PiRGBArray(camera, size=RESOLUTION)
        for capture in camera.capture_continuous(raw_capture, format='bgr', use_video_port=True):
            if enable_laser:
                camera.shutter_speed = 500
                enable_laser = False
            elif enable_fiducial:
                camera.shutter_speed = 0
                enable_fiducial = False

            frame = capture.array
            # do tracking
            find_markers(frame)
            find_laser(frame)
            raw_capture.truncate(0)
            if debug:
                print(get_error(), laser_x)
            if kill_loop:
                break

def end_tracking_loop():
    global kill_loop
    kill_loop = True

def get_error():
    return error_x, error_y

if __name__ == '__main__':    
    run_tracking_loop(debug=True)
