import tkinter
from numpy import core
from numpy.lib.function_base import vectorize
from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from kinematics import corexy_inverse, corexy_transform, z0, z1, z2, center_home
import kinematics
from marker_tracking import get_laser_displacement, run_tracking_loop, get_error, end_tracking_loop, enable_fiducial_sensing, enable_laser_sensing
from threading import Thread
import numpy as np
import math
from time import time, sleep
from queue import Queue
import os
from xbox360controller import Xbox360Controller
from gcode_solver import GcodeSolver
from pinc_state import State
import logging

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

state = None
event_queue = Queue()

def post_event(event):
    event_queue.put(event)


def handle_events():
    global state
    if not event_queue.empty():
        event = event_queue.get()
        logging.info(event)
        state = state.on_event(event)


class InitState(State):
    def __init__(self):
        super().__init__()
        self.event_map['init'] = HomeState

    def run(self):
        post_event('init')

class JogState(State):
    def __init__(self):
        super().__init__()
    
    def run(self):
        self.xpos, self.ypos = corexy_inverse(embedded_motors[3].theta - FineHomeState.home_3, embedded_motors[4].theta- FineHomeState.home_4)
        self.xvel, self.yvel = corexy_inverse(embedded_motors[3].omega, embedded_motors[4].omega)

class HomeState(State):
    def __init__(self):
        super().__init__()
        self.event_map['found home'] = FineHomeState
        enable_fiducial_sensing()
        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.DISABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
        write(control_packet)

    def run(self):
        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=1)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=HOMING_SPEED)
        write(control_packet)

        e_x, e_y = get_error()
        if e_x is not None or e_y is not None:
            post_event('found home')


class FineHomeState(State):
    home_3 = 0
    home_4 = 0
    def __init__(self):
        super().__init__()
        self.event_map['fine home complete'] = JogHomeCenterState
        self.event_map['lost tracking'] = HomeState
        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
        write(control_packet)

    def run(self):
        global home_x, home_y
        e_x, e_y = get_error()
        if e_x is None or e_y is None:
            post_event("lost tracking")
            return
        errorx = -e_x + e_y
        errory = -e_x - e_y
        control_packet = pkt.pack_HeaderPacket(
            pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=-errory/10)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=-errorx/10)
        write(control_packet)

        if np.sqrt(errorx**2 + errory**2) < 1:
            FineHomeState.home_4 = embedded_motors[4].theta
            FineHomeState.home_3 = embedded_motors[3].theta
            enable_laser_sensing()
            post_event('fine home complete')

class JogHomeState(JogState):
    def __init__(self):
        super().__init__()
        self.home_event = 'at home'
        self.home_x = 30
        self.home_y = 30

    def run(self):
        super().run()
        errorx = self.home_x-self.xpos
        errory = self.home_y-self.ypos
        control_x = errorx*20 - self.xvel*10
        control_y = errory*20 - self.yvel*20
        control_3, control_4 = corexy_transform(control_x, control_y)
        control_packet = pkt.pack_HeaderPacket(
            pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[3].motorId, pkt.MotorCommand.SET_ALPHA, control=control_3)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[4].motorId, pkt.MotorCommand.SET_ALPHA, control=control_4)

        write(control_packet)
        if np.sqrt(errorx**2 + errory**2) < .005:
            post_event(self.home_event)

class HomeZState(State):
    def __init__(self):
        super().__init__()

        self.motor_index = 'all'
        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=5)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[2].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[1].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[0].motorId, pkt.MotorCommand.ENABLE)
        write(control_packet)

    def run(self):
        z_nominal = np.clip(get_laser_displacement()/10, -10, 10)
        if self.motor_index == 'all':
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
        else:
            control_packet = pkt.pack_HeaderPacket(
                pkt.SerialCommand.RUN_MOTOR, motorCount=5)
            for motor in embedded_motors:
                if embedded_motors[motor].motorId != self.motor_index:
                    control_packet += pkt.pack_MotorCommandPacket(
                        embedded_motors[motor].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
                else:
                    control_packet += pkt.pack_MotorCommandPacket(
                        embedded_motors[self.motor_index].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
            write(control_packet)

        if z_nominal == 0:
            post_event('z home')
            home_z = embedded_motors[0].theta


class JogHomeCenterState(JogHomeState):
    def __init__(self):
        super().__init__()
        self.event_map['at home'] = HomeCenterState
        self.home_x = center_home[0]
        self.home_y = center_home[1]


class HomeCenterState(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome0State


class JogHome0State(JogHomeState):
    def __init__(self):
        super().__init__()
        self.event_map['at home'] = HomeZ0State
        self.home_x = z0[0]
        self.home_y = z0[1]


class HomeZ0State(HomeZState):
    def __init__(self):
        super().__init__()
        self.motor_index = 0
        self.event_map['z home'] = JogHome1State


class JogHome1State(JogHomeState):
    def __init__(self):
        super().__init__()
        self.event_map['at home'] = HomeZ1State
        self.home_x = z1[0]
        self.home_y = z1[1]


class HomeZ1State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome2State
        self.motor_index = 1


class JogHome2State(JogHomeState):
    def __init__(self):
        super().__init__()
        self.event_map['at home'] = HomeZ2State
        self.home_x = z2[0]
        self.home_y = z2[1]


class HomeZ2State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = Jog00State
        self.motor_index = 2

class Jog00State(JogHomeState):
    def __init__(self):
        super().__init__()
        end_tracking_loop()
        self.event_map['at home'] = PrintState
        self.home_x = 0
        self.home_y = 0

class PrintState(JogState):
    def __init__(self):
        super().__init__()
        self.start_time = time()
    
    def run(self):
        super().run()
        KP = 5000
        KP_VELOCITY = 1000
        positions, velocities = path_planner.get_solution(time()-self.start_time)
        position = positions[0]
        x_nominal = position[0]/XY_MM_PER_RAD
        y_nominal = position[1]/XY_MM_PER_RAD
        z_nominal = position[2]/Z_MM_PER_RAD
        x_velocity_nominal = velocities[0]/XY_MM_PER_RAD
        y_velocity_nominal = velocities[1]/XY_MM_PER_RAD
        z_velocity_nominal = velocities[2]/Z_MM_PER_RAD
        v_errorx = x_velocity_nominal - self.xvel
        v_errory = y_velocity_nominal - self.yvel

        errorx = x_nominal - self.xpos
        control_inputx = np.clip(KP*errorx + KP_VELOCITY*v_errorx, -MAX_ACCELERATION, MAX_ACCELERATION)
        
        errory = y_nominal - self.ypos
        control_inputy = np.clip(KP*errory + KP_VELOCITY*v_errory, -MAX_ACCELERATION, MAX_ACCELERATION)

        control3, control4 = corexy_transform(control_inputx, control_inputy)

        # errorz2 = z_nominal - embedded_motors[2].theta
        # control_inputz2 = KP*errorz2 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[2].omega)
        # errorz1 = z_nominal - embedded_motors[1].theta
        # control_inputz1 = KP*errorz1 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[1].omega)
        # errorz0 = z_nominal - embedded_motors[0].theta
        # control_inputz0 = KP*errorz0 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[0].omega)

        control_packet = pkt.pack_HeaderPacket(
            pkt.SerialCommand.RUN_MOTOR, motorCount=5)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[4].motorId, pkt.MotorCommand.SET_ALPHA, control=control4)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[3].motorId, pkt.MotorCommand.SET_ALPHA, control=control3)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[2].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[1].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[0].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
        write(control_packet)


class ManualState(State):
    Z_JOG = 10
    XY_JOG = 20
    def __init__(self):
        super().__init__()
        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=5)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[2].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[1].motorId, pkt.MotorCommand.ENABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[0].motorId, pkt.MotorCommand.ENABLE)
        write(control_packet)

    def run(self):
        if jog_controller.button_a.is_pressed:
            z_nominal = (controller.trigger_l.value - controller.trigger_r.value)
            if abs(z_nominal) < .2:
                z_nominal = 0
            z_nominal*= ManualState.Z_JOG
            x_nominal = controller.axis_l.x
            if abs(x_nominal) < .2:
                x_nominal = 0
            x_nominal*= ManualState.XY_JOG
            y_nominal = controller.axis_l.y
            if abs(y_nominal) < .2:
                y_nominal = 0
            y_nominal*= ManualState.XY_JOG
        else:
            z_nominal = 0
            y_nominal = 0
            x_nominal = 0
        
        motor_3_control, motor_4_control = corexy_transform(x_nominal, y_nominal)
        control_packet = pkt.pack_HeaderPacket(
            pkt.SerialCommand.RUN_MOTOR, motorCount=5)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[2].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[1].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[0].motorId, pkt.MotorCommand.SET_OMEGA, control=z_nominal)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=motor_3_control)
        control_packet += pkt.pack_MotorCommandPacket(
            embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=motor_4_control)
        
        write(control_packet)


def embedded_service():
    global errorx, errory, x_velocity_nominal, y_velocity_nominal, v_errorx, v_errory, current_state, home_x, home_y, home_z, z_velocity_nominal, state
    KP = 5000
    KP_VELOCITY = 1000
    KD = 0
    count = 0
    start_time = time()
    first_message = True
    state = InitState()
    
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
        handle_events()
        state.run()
        # continue

        # if current_state != "RUNNING":
        #     positions, velocities = path_planner.get_solution(0)
        # else:
        #     positions, velocities = path_planner.get_solution(time()-start_time)

        # position = positions[0]
        # x_nominal = position[0]/XY_MM_PER_RAD + home_x
        # y_nominal = position[1]/XY_MM_PER_RAD + home_y
        # z_nominal = position[2]/Z_MM_PER_RAD + home_z
        # x_velocity_nominal = velocities[0]/XY_MM_PER_RAD
        # y_velocity_nominal = velocities[1]/XY_MM_PER_RAD
        # z_velocity_nominal = velocities[2]/Z_MM_PER_RAD
        # v_errorx = x_velocity_nominal - embedded_motors[4].omega
        # v_errory = y_velocity_nominal - embedded_motors[3].omega


        # else:
        #     errorx = x_nominal - embedded_motors[4].theta
        #     control_inputx = np.clip(KP*errorx - KD*embedded_motors[4].omega + KP_VELOCITY*v_errorx, -MAX_ACCELERATION, MAX_ACCELERATION)
            
        #     errory = y_nominal - embedded_motors[3].theta
        #     control_inputy = np.clip(KP*errory - KD*embedded_motors[3].omega + KP_VELOCITY*v_errory, -MAX_ACCELERATION, MAX_ACCELERATION)

        #     errorz2 = z_nominal - embedded_motors[2].theta
        #     control_inputz2 = KP*errorz2 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[2].omega)
        #     errorz1 = z_nominal - embedded_motors[1].theta
        #     control_inputz1 = KP*errorz1 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[1].omega)
        #     errorz0 = z_nominal - embedded_motors[0].theta
        #     control_inputz0 = KP*errorz0 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[0].omega)

        #     control_packet = pkt.pack_HeaderPacket(
        #         pkt.SerialCommand.RUN_MOTOR, motorCount=5)
        #     control_packet += pkt.pack_MotorCommandPacket(
        #         embedded_motors[4].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputx)
        #     control_packet += pkt.pack_MotorCommandPacket(
        #         embedded_motors[3].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputy)
        #     control_packet += pkt.pack_MotorCommandPacket(
        #         embedded_motors[2].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputz2)
        #     control_packet += pkt.pack_MotorCommandPacket(
        #         embedded_motors[1].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputz1)
        #     control_packet += pkt.pack_MotorCommandPacket(
        #         embedded_motors[0].motorId, pkt.MotorCommand.SET_ALPHA, control=control_inputz0)
        #     write(control_packet)


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
            sleep(.25)
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

            # print(get_laser_displacement(), get_error(), state, controller.trigger_l.value)
            # print(corexy_inverse(embedded_motors[3].theta - FineHomeState.home_3, embedded_motors[4].theta- FineHomeState.home_4))
