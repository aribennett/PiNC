import tkinter
from numpy.lib.function_base import vectorize
from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from marker_tracking import get_laser_displacement, run_tracking_loop, get_error, end_tracking_loop
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
Z_MM_PER_RAD = 0.795774715
HOMING_SPEED = 10

x_nominal = 0
y_nominal = 0
z_nominal = 0

home_x = 0
home_y = 0
home_z = 0
errorx = 0
errory = 0
v_errorx = 0
v_errory = 0

x_velocity_nominal = 0
y_velocity_nominal = 0
z_velocity_nominal = 0
start_time = None
embedded_motors = {}
embedded_sensors = {}
jog_controller = None

# current_state = "MANUAL_CONTROL"
current_state = "HOMING"

with open('box_gcode.gcode', 'r') as f:
  gcode = f.read()

path_planner = GcodeSolver(gcode)

def embedded_service():
    global errorx, errory, x_velocity_nominal, y_velocity_nominal, v_errorx, v_errory, current_state, home_x, home_y, home_z, z_velocity_nominal
    KP = 5000
    KP_VELOCITY = 1000
    KD = 0
    count = 0
    start_time = time()
    first_message = True
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

        if first_message:
            first_message = False
            if current_state == "HOMING":
                control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=1)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
                write(control_packet)
            elif current_state == "FINE_HOME":
                control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=2)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
                write(control_packet)
            elif current_state == "MANUAL_CONTROL":
                control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=5)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[2].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[1].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[0].motorId, pkt.MotorCommand.ENABLE)
                write(control_packet)
            elif current_state == "HOME_Z":
                control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=5)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[2].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[1].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[0].motorId, pkt.MotorCommand.ENABLE)
                write(control_packet)
            else:
                start_time = time()
                control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=5)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[2].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[1].motorId, pkt.MotorCommand.ENABLE)
                control_packet += pkt.pack_MotorCommandPacket(embedded_motors[0].motorId, pkt.MotorCommand.ENABLE)
                write(control_packet)

        if current_state != "RUNNING":
            positions, velocities = path_planner.get_solution(0)
        else:
            positions, velocities = path_planner.get_solution(time()-start_time)

        position = positions[0]
        x_nominal = position[0]/XY_MM_PER_RAD + home_x
        y_nominal = position[1]/XY_MM_PER_RAD + home_y
        z_nominal = position[2]/Z_MM_PER_RAD + home_z
        x_velocity_nominal = velocities[0]/XY_MM_PER_RAD
        y_velocity_nominal = velocities[1]/XY_MM_PER_RAD
        z_velocity_nominal = velocities[2]/Z_MM_PER_RAD
        v_errorx = x_velocity_nominal - embedded_motors[4].omega
        v_errory = y_velocity_nominal - embedded_motors[3].omega

        e_x, e_y = get_error()

        if (e_x is None or e_y is None) and current_state not in ["JOG_HOME", "RUNNING"]:
            if current_state == "HOMING":
                errory = 100
                errorx = 0
        elif current_state == "HOMING":
            e_x = e_y = 0
            current_state = "FINE_HOME"
            first_message = True
            print("Saw home")
            continue
        elif current_state == "FINE_HOME":
            errorx = -e_x + e_y
            errory = -e_x - e_y
            if np.sqrt(errorx**2 + errory**2) < 1:
                print("Homed")
                current_state = "JOG_HOME"
                # end_tracking_loop()
                home_x = embedded_motors[4].theta
                home_y = embedded_motors[3].theta
                print(home_x, home_y)
                continue
        elif current_state == "JOG_HOME":
            errorx = -(embedded_motors[4].theta - home_x)
            errory = (-300/XY_MM_PER_RAD + home_y) - embedded_motors[3].theta
            if np.sqrt(errorx**2 + errory**2) < .005:
                print("Jogged to home")
                # current_state = "RUNNING"
                current_state = "HOME_Z"
                first_message = True
                home_x = embedded_motors[4].theta
                home_y = embedded_motors[3].theta
                print(home_x, home_y)
                continue

        
        if current_state == "HOMING":
            control_packet = pkt.pack_HeaderPacket(
                pkt.SerialCommand.RUN_MOTOR, motorCount=1)
            control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=HOMING_SPEED)
            write(control_packet)
        elif current_state == "FINE_HOME":
            control_packet = pkt.pack_HeaderPacket(
                pkt.SerialCommand.RUN_MOTOR, motorCount=2)
            control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=-errory/10)
            control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=-errorx/10)
            write(control_packet)
        elif current_state == "JOG_HOME":
            control_packet = pkt.pack_HeaderPacket(
                pkt.SerialCommand.RUN_MOTOR, motorCount=2)
            control_inputx = np.clip(50*errorx - 20*embedded_motors[4].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
            control_inputy = np.clip(50*errory - 20*embedded_motors[3].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[4].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputx)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[3].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputy)
            write(control_packet)
        elif current_state == "HOME_Z":
            z_nominal = np.clip(get_laser_displacement()/10, -10, 10)
            control_packet = pkt.pack_HeaderPacket(
                pkt.SerialCommand.RUN_MOTOR, motorCount=5)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[2].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[1].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[0].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            write(control_packet)
            if z_nominal == 0:
                current_state = "MANUAL_CONTROL"
                first_message = True
                home_z = embedded_motors[0].theta
        elif current_state == "MANUAL_CONTROL":
            if jog_controller.button_a.is_pressed:
                z_nominal = 20
            elif jog_controller.button_b.is_pressed:
                z_nominal = -20
            else:
                z_nominal = 0
            control_packet = pkt.pack_HeaderPacket(
                pkt.SerialCommand.RUN_MOTOR, motorCount=3)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[2].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[1].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[0].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            write(control_packet)
        else:
            errorx = x_nominal - embedded_motors[4].theta
            control_inputx = np.clip(KP*errorx - KD*embedded_motors[4].omega + KP_VELOCITY*v_errorx, -MAX_ACCELERATION, MAX_ACCELERATION)
            
            errory = y_nominal - embedded_motors[3].theta
            control_inputy = np.clip(KP*errory - KD*embedded_motors[3].omega + KP_VELOCITY*v_errory, -MAX_ACCELERATION, MAX_ACCELERATION)

            errorz2 = z_nominal - embedded_motors[2].theta
            control_inputz2 = KP*errorz2 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[2].omega)
            errorz1 = z_nominal - embedded_motors[1].theta
            control_inputz1 = KP*errorz1 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[1].omega)
            errorz0 = z_nominal - embedded_motors[0].theta
            control_inputz0 = KP*errorz0 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[0].omega)

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
    cold_start('/dev/hidraw0')
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        start_time = time()
        jog_controller = controller
        tracking_thread = Thread(target=run_tracking_loop, daemon=True)
        tracking_thread.start()
        print("Started tracking")
        sleep(2)
        embedded_thread = Thread(target=embedded_service, daemon=True)
        embedded_thread.start()
        print("Started controls")
        while True:
            sleep(1)
            # print(embedded_motors[4].theta, embedded_motors[3].theta, errorx, errory)
            # print(-100/XY_MM_PER_RAD, embedded_motors[3].theta, home_y)
            # print((-100/XY_MM_PER_RAD + home_y) - embedded_motors[3].theta)
            # print(current_state)
            # pos_error = math.sqrt(errorx**2 + errory**2)*XY_MM_PER_RAD
            # vel_error = math.sqrt(v_errorx**2 + v_errory**2)*XY_MM_PER_RAD
            # print(str(pos_error).ljust(30, ' '), v_errorx, v_errory)
            # print(str(pos_error))
            # pos, vel = path_planner.get_solution(time()-start_time)
            # print(pos[0], vel)

            print(get_laser_displacement(), current_state)
