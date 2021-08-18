# Autogenerated Packet definitions. See generate_header.py
from collections import namedtuple
from struct import unpack, pack

class SerialCommand():
  REPORT_STATUS = 1
  GET_STATUS = 2
  RUN_MOTOR = 3
  RESET = 4

class MotorCommand():
  STATUS = 1
  SET_OMEGA = 2
  SET_ALPHA = 3
  SET_THETA = 4

class SerialState():
  IDLE = 0
  WAITING_FOR_HEADER = 1
  WAITING_FOR_BODY = 2

HeaderPacket = namedtuple('HeaderPacket', 'command motorCount sensorCount ')
MotorPacket = namedtuple('MotorPacket', 'motorId motorCommand theta omega alpha ')
SensorPacket = namedtuple('SensorPacket', 'sensorID sensorValue ')
size_HeaderPacket = 3
size_MotorPacket = 14
size_SensorPacket = 5

def unpack_HeaderPacket(bytes):
    return HeaderPacket._make(unpack('=BBB',bytes))

def pack_HeaderPacket(command=0, motorCount=0, sensorCount=0):
    return pack('=BBB', command, motorCount, sensorCount)

def unpack_MotorPacket(bytes):
    return MotorPacket._make(unpack('=BBfff',bytes))

def pack_MotorPacket(motorId=0, motorCommand=0, theta=0, omega=0, alpha=0):
    return pack('=BBfff', motorId, motorCommand, theta, omega, alpha)

def unpack_SensorPacket(bytes):
    return SensorPacket._make(unpack('=Bl',bytes))

def pack_SensorPacket(sensorID=0, sensorValue=0):
    return pack('=Bl', sensorID, sensorValue)
