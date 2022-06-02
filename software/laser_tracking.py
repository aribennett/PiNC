import cv2
import numpy as np
from numpy.core.defchararray import greater
from picamera import PiCamera
from picamera.array import PiRGBArray
RESOLUTION = (400, 400)
FRAMERATE = 60

kill_loop = False
laser_x = 0


def get_laser_displacement():
    return laser_x-RESOLUTION[0]/2


def run_tracking_loop(debug=False):
    global kill_loop, laser_x
    kill_loop = False
    with PiCamera() as camera:
        camera.resolution = RESOLUTION
        camera.framerate = FRAMERATE
        camera.shutter_speed = 100
        raw_capture = PiRGBArray(camera, size=RESOLUTION)
        for raw in camera.capture_continuous(raw_capture, format='bgr', use_video_port=True):
            image = raw.array.copy()
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower = np.array([0, 0, 0])
            upper = np.array([255, 255, 255])
            mask = cv2.inRange(image, lower, upper)
            M = cv2.moments(mask)
            # calculate x,y coordinate of center
            if M["m00"] != 0:
                laser_x = int(M["m01"] / M["m00"])
            else:
                laser_x = 0
            if debug:
                print(laser_x)
            raw_capture.truncate(0)
            if kill_loop:
                break


def end_tracking_loop():
    global kill_loop
    kill_loop = True


if __name__ == '__main__':
    run_tracking_loop(debug=True)
