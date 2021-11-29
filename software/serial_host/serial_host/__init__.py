from glob import glob
import os
import logging

is_windows = os.name == 'nt'
if is_windows:
    import hid

pid = 0x0486
vid = 0x16c0
device = None

error_count = 0
MAX_SEND_ERRORS = 10
HID_ENDPOINT_SIZE = 1024


def cold_start(device_name=None):
    global device
    if is_windows:
        device = hid.Device(vid, pid)
    else:
        flags = os.O_RDWR
        device = os.open(device_name, flags)
    pass


def write(packet):
    global error_count
    try:
        out = b'\00' + packet  # Teensy eats one byte
        if is_windows:
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
    if is_windows:
        return device.read(HID_ENDPOINT_SIZE, 1000)
    else:
        return os.read(device, HID_ENDPOINT_SIZE)

