# Autogenerated Packet definitions. See generate_header.py
from collections import namedtuple
from struct import unpack, pack

class SerialCommand():
  REPORT_STATUS = 1
  GET_STATUS = 2
  RUN_MOTOR = 3
  RESET = 4

class MotorCommand():
  NONE = 1
  SET_OMEGA = 2
  SET_ALPHA = 3
  SET_THETA = 4

HeaderPacket = namedtuple('HeaderPacket', 'command motorCount sensorCount ')
MotorStatePacket = namedtuple('MotorStatePacket', 'motorId theta omega ')
MotorCommandPacket = namedtuple('MotorCommandPacket', 'motorId motorCommand control ')
SensorPacket = namedtuple('SensorPacket', 'sensorID sensorValue ')
size_HeaderPacket = 3
size_MotorStatePacket = 9
size_MotorCommandPacket = 6
size_SensorPacket = 3

def unpack_HeaderPacket(bytes):
    return HeaderPacket._make(unpack('=BBB',bytes))

def pack_HeaderPacket(command=0, motorCount=0, sensorCount=0):
    return pack('=BBB', command, motorCount, sensorCount)

def unpack_MotorStatePacket(bytes):
    return MotorStatePacket._make(unpack('=Bff',bytes))

def pack_MotorStatePacket(motorId=0, theta=0, omega=0):
    return pack('=Bff', motorId, theta, omega)

def unpack_MotorCommandPacket(bytes):
    return MotorCommandPacket._make(unpack('=BBf',bytes))

def pack_MotorCommandPacket(motorId=0, motorCommand=0, control=0):
    return pack('=BBf', motorId, motorCommand, control)

def unpack_SensorPacket(bytes):
    return SensorPacket._make(unpack('=Bh',bytes))

def pack_SensorPacket(sensorID=0, sensorValue=0):
    return pack('=Bh', sensorID, sensorValue)