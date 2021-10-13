import tkinter
from numpy.lib.function_base import vectorize
from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from threading import Thread
import numpy as np
import math
from time import time, sleep
import os
from xbox360controller import Xbox360Controller
from gcode_solver import GcodeSolver
CONTROLLER_DEAD_ZONE = 0.2
CONTROLLER_JOG_RATE = 100
MAX_ACCELERATION = 1000
XY_MM_PER_RAD = 6.36619783227

x_nominal = 0
y_nominal = 0
z_nominal = 0
errorx = 0
errory = 0
v_errorx = 0
v_errory = 0
x_velocity_nominal = 0
y_velocity_nominal = 0
start_time = None
embedded_motors = {}
embedded_sensors = {}
jog_controller = None

with open('box_gcode.gcode', 'r') as f:
  gcode = f.read()

path_planner = GcodeSolver(gcode)

def embedded_service():
    global errorx, errory, x_velocity_nominal, y_velocity_nominal, v_errorx, v_errory
    KP = 5000
    KP_VELOCITY = 1000
    KD = 0
    KI = 0
    count = 0
    start_time = time()
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

        if jog_controller.button_trigger_l.is_pressed:
            # if jog_controller.axis_l.x > CONTROLLER_DEAD_ZONE:
            #     x_nominal = (jog_controller.axis_l.x-CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            # elif jog_controller.axis_l.x < -CONTROLLER_DEAD_ZONE:
            #     x_nominal = (jog_controller.axis_l.x+CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            # else:
            #     x_nominal = 0

            # if jog_controller.axis_l.y > CONTROLLER_DEAD_ZONE:
            #     y_nominal = (-jog_controller.axis_l.y+CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            # elif jog_controller.axis_l.y < -CONTROLLER_DEAD_ZONE:
            #     y_nominal = (-jog_controller.axis_l.y-CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            # else:
            #     y_nominal = 0
            
            z_nominal = (jog_controller.trigger_l.value - jog_controller.trigger_r.value)*CONTROLLER_JOG_RATE/3
            
        else:
            # x_nominal = 0
            # y_nominal = 0
            z_nominal = 0
        # RADIUS = 10
        # SPEED = .5
        # x_nominal = math.sin(SPEED*time())*RADIUS
        # y_nominal = math.cos(SPEED*time())*RADIUS
        positions, velocities = path_planner.get_solution(time()-start_time)
        position = positions[0]
        x_nominal = position[0]/XY_MM_PER_RAD
        y_nominal = position[1]/XY_MM_PER_RAD
        x_velocity_nominal = velocities[0]/XY_MM_PER_RAD
        y_velocity_nominal = velocities[1]/XY_MM_PER_RAD
        v_errorx = x_velocity_nominal - embedded_motors[4].omega
        v_errory = y_velocity_nominal - embedded_motors[3].omega

        errorx = x_nominal - embedded_motors[4].theta
        control_inputx = np.clip(KP*errorx - KD*embedded_motors[4].omega + KP_VELOCITY*v_errorx, -MAX_ACCELERATION, MAX_ACCELERATION)

        errory = y_nominal - embedded_motors[3].theta
        control_inputy = np.clip(KP*errory - KD*embedded_motors[3].omega + KP_VELOCITY*v_errory, -MAX_ACCELERATION, MAX_ACCELERATION)

        errorz2 = z_nominal - embedded_motors[2].omega
        control_inputz2 = 10*errorz2
        errorz1 = z_nominal - embedded_motors[1].omega
        control_inputz1 = 10*errorz1
        errorz0 = z_nominal - embedded_motors[0].omega
        control_inputz0 = 10*errorz0

        control_packet = pkt.pack_HeaderPacket(
            pkt.SerialCommand.RUN_MOTOR, motorCount=5)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[4].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputx)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[3].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputy)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[2].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputz2)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[1].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputz1)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[0].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputz0)
        write(control_packet)


if __name__ == "__main__":
    os.system(f"taskset -p -c 3 {os.getpid()}")
    cold_start('/dev/hidraw2')
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        start_time = time()
        jog_controller = controller
        embedded_thread = Thread(target=embedded_service, daemon=True)
        embedded_thread.start()
        while True:
            sleep(.1)
            # pos_error = math.sqrt(errorx**2 + errory**2)*XY_MM_PER_RAD
            # vel_error = math.sqrt(v_errorx**2 + v_errory**2)*XY_MM_PER_RAD
            # print(str(pos_error).ljust(30, ' '), v_errorx, v_errory)
            # print(str(pos_error))
            # pos, vel = path_planner.get_solution(time()-start_time)
            # print(pos[0], vel)
