import os

is_windows = os.name == 'nt'
if is_windows:
  import hid

pid = 0x0486
vid = 0x16c0
device = None


def cold_start(device_name=None):
  global device
  if is_windows:
    device = hid.Device(vid, pid)
  else:
    flags = os.O_RDWR
    device = os.open('/dev/hidraw0', flags)
  pass


def write(packet):
  out = b'\00' + packet  # Teensy eats one byte
  if is_windows:
    device.write(out)
  else:
    os.write(device, out)


def read():
  if is_windows:
    return device.read(64, 1000)
  else:
    return os.read(device, 64)
