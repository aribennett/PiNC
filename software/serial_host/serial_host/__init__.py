from glob import glob
from sys import platform
import os
import logging

device = None

error_count = 0
MAX_SEND_ERRORS = 10
HID_ENDPOINT_SIZE = 64


def cold_start():
    global device
    files = glob('/dev/pinc*')
    if len(files) < 1:
        raise ValueError("No MCU found")
    flags = os.O_RDWR
    device = os.open(files[0], flags)


def close_device():
    device.close()


def write(packet):
    global error_count
    try:
        out = b'\00' + packet  # Teensy eats one byte
        os.write(device, out)
        error_count = 0
    except:
        logging.error("Write Exception")
        error_count += 1

    if error_count >= MAX_SEND_ERRORS:
        raise Exception("Too many failed sends")


def read():
    return os.read(device, HID_ENDPOINT_SIZE)
