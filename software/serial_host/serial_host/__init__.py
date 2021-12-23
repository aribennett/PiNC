from glob import glob
from sys import platform
import os
import logging

is_linux = platform == "linux" or platform == "linux2"

if not is_linux:
    import hid

device = None

error_count = 0
MAX_SEND_ERRORS = 10
HID_ENDPOINT_SIZE = 64


def cold_start(device_name=None, vid=0x0000, pid=0x0000):
    global device
    if is_linux:
        flags = os.O_RDWR
        device = os.open(device_name, flags)
    else:
        device = hid.Device(vid, pid)

def close_device():
    device.close()

def write(packet):
    global error_count
    try:
        out = b'\00' + packet  # Teensy eats one byte
        if not is_linux:
            device.write(out)
        else:
            os.write(device, out)
        error_count = 0
    except:
        logging.error("Write Exception")
        error_count += 1

    if error_count >= MAX_SEND_ERRORS:
        raise Exception("Too many failed sends")

def read():
    if not is_linux:
        return device.read(HID_ENDPOINT_SIZE, 1000)
    else:
        return os.read(device, HID_ENDPOINT_SIZE)

