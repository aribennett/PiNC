from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from threading import Thread
import numpy as np
from time import time, sleep
from inputs import get_gamepad
import os

MAX_ACCELERATION = 1000
ROTATION_SPEED = 5
DEADBAND = 0.4

x_nominal = 0
y_nominal = 0
z_nominal = 0
theta_nominal = 90

embedded_motors = {}
embedded_sensors = {}
jog_controller = None

def handleSlider(value):
    global x_nominal
    x_nominal = float(value)

def handleSlider2(value):
    global y_nominal
    y_nominal = float(value)

def handleSlider3():
    global z_nominal
    if z_nominal != 0:
        z_nominal = 0
    else:
        z_nominal = 15

def handleSlider4():
    global theta_nominal
    if z_nominal > 0:
        theta_nominal = 0
        sleep(.5)
        theta_nominal = 90

def embedded_service():
    packet_count = 0
    KP = 200
    KD = 10
    KI = 0
    while True:
        hid_msg = read()
        header = pkt.unpack_HeaderPacket(hid_msg[:pkt.size_HeaderPacket])
        unpack_index = pkt.size_HeaderPacket
        for i in range(header.motorCount):
            motor_packet = pkt.unpack_MotorStatePacket(
                hid_msg[unpack_index:unpack_index+pkt.size_MotorStatePacket])
            embedded_motors[motor_packet.motorId] = motor_packet
            unpack_index += pkt.size_MotorStatePacket
        for i in range(header.sensorCount):
            sensor_packet = pkt.unpack_SensorPacket(
                hid_msg[unpack_index:unpack_index+pkt.size_SensorPacket])
            embedded_sensors[sensor_packet.sensorId] = sensor_packet
            unpack_index += pkt.size_SensorPacket

        if abs(x_nominal/ROTATION_SPEED) > DEADBAND:
            errorx = x_nominal - embedded_motors[0].omega
        else:
            errorx = -embedded_motors[0].omega

        control_inputx = np.clip(KP*errorx - KD*embedded_motors[0].omega, -MAX_ACCELERATION, MAX_ACCELERATION)

        if abs(y_nominal/ROTATION_SPEED) > DEADBAND:
            errory = y_nominal - embedded_motors[1].omega
        else:
            errory = -embedded_motors[1].omega
        control_inputy = np.clip(KP*errory - KD*embedded_motors[1].omega, -MAX_ACCELERATION, MAX_ACCELERATION)

        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=4)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[0].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputx)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[1].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputy)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[2].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.SET_THETA, control=theta_nominal)
        write(control_packet)


if __name__ == "__main__":
    cold_start()
    embedded_thread = Thread(target=embedded_service, daemon=True)
    embedded_thread.start()

    while True:
        events = get_gamepad()
        for event in events:
            if event.code == "ABS_X":
                y_nominal = ROTATION_SPEED*event.state/32768
            if event.code == "ABS_Y":
                x_nominal = -ROTATION_SPEED*event.state/32768
            if event.code == "BTN_TL":
                z_nominal = 45*event.state
            if event.code == "BTN_TR":
                if z_nominal > 0:
                    theta_nominal = 90-90*event.state
                else:
                    theta_nominal = 90

