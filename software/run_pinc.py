from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from kinematics import corexy_inverse, corexy_transform, z0, z1, z2, center_home
from marker_tracking import get_laser_displacement, run_tracking_loop, get_error, end_tracking_loop, enable_fiducial_sensing, enable_laser_sensing
from threading import Thread
import numpy as np
from time import time, sleep
from queue import Queue
import os
from xbox360controller import Xbox360Controller
from pinc_state import State
import sys
import logging

CONTROLLER_DEAD_ZONE = 0.2
CONTROLLER_JOG_RATE = 100
MAX_ACCELERATION = 1000
XY_MM_PER_RAD = 6.36619783227
Z_MM_PER_RAD = 0.795774715
HOMING_SPEED = 10

# ------ Debug Variables --------
errorx = 0
errory = 0
# -------------------------------

embedded_motors = {}
embedded_sensors = {}
jog_controller = None

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
        # self.event_map['init'] = HomeState
        self.event_map['init'] = ManualState

    def run(self):
        post_event('init')


class JogState(State):
    def __init__(self):
        super().__init__()
        self.xstart, self.ystart = corexy_inverse(embedded_motors[3].theta - FineHomeState.home_3, embedded_motors[4].theta- FineHomeState.home_4)
        self.jog_time = 10
        self.start_time = time()
        self.x_target, self.y_target = 0, 0
        self.xw_nominal, self.yw_nominal = 0, 0

    def set_jog_target(self, x, y, time):
        self.jog_time = time
        self.x_target, self.y_target = x, y
        self.xw_nominal = (self.x_target - self.xstart)/self.jog_time
        self.yw_nominal = (self.y_target - self.ystart)/self.jog_time

    def run(self):
        self.xpos, self.ypos = corexy_inverse(embedded_motors[3].theta - FineHomeState.home_3, embedded_motors[4].theta- FineHomeState.home_4)
        self.xvel, self.yvel = corexy_inverse(embedded_motors[3].omega, embedded_motors[4].omega)

        interp = (time()-self.start_time)/self.jog_time
        if interp >= 1:
            control_packet = pkt.pack_HeaderPacket(
                command=pkt.SerialCommand.RUN_MOTOR, motorCount=2)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=0)
            post_event('jog done')
        else:
            x_nominal = interp*(self.x_target - self.xstart) + self.xstart
            y_nominal = interp*(self.y_target - self.ystart) + self.ystart
            x_error = x_nominal - self.xpos
            y_error = y_nominal - self.ypos
            control_x = x_error + self.xw_nominal
            control_y = y_error + self.yw_nominal
            control_3, control_4 = corexy_transform(control_x, control_y)
            control_packet = pkt.pack_HeaderPacket(
                command=pkt.SerialCommand.RUN_MOTOR, motorCount=2)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=control_3)
            control_packet += pkt.pack_MotorCommandPacket(
                embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=control_4)
        write(control_packet)


class HomeState(State):
    def __init__(self):
        super().__init__()
        self.event_map['found home'] = FineHomeState
        enable_fiducial_sensing()
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.DISABLE)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.ENABLE)
        write(control_packet)

    def run(self):
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=1)
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
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=2)
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
            command=pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[3].motorId, pkt.MotorCommand.SET_OMEGA, control=-errory/10)
        control_packet += pkt.pack_MotorCommandPacket(embedded_motors[4].motorId, pkt.MotorCommand.SET_OMEGA, control=-errorx/10)
        write(control_packet)

        if np.sqrt(errorx**2 + errory**2) < 1:
            enable_laser_sensing()
            FineHomeState.home_4 = embedded_motors[4].theta
            FineHomeState.home_3 = embedded_motors[3].theta
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
            command=pkt.SerialCommand.RUN_MOTOR, motorCount=2)
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
        enable_laser_sensing()
        self.motor_index = 'all'
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=5)
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
                command=pkt.SerialCommand.RUN_MOTOR, motorCount=5)
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
                command=pkt.SerialCommand.RUN_MOTOR, motorCount=5)
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


class JogHomeCenterState(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeCenterState
        self.set_jog_target(30, 30, 5)


class HomeCenterState(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome0State


class JogHome0State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ0State
        self.set_jog_target(z0[0], z0[1], 5)


class HomeZ0State(HomeZState):
    def __init__(self):
        super().__init__()
        self.motor_index = 0
        self.event_map['z home'] = JogHome1State


class JogHome1State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ1State
        self.set_jog_target(z1[0], z1[1], 5)

class HomeZ1State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome2State
        self.motor_index = 1


class JogHome2State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ2State
        self.set_jog_target(z2[0], z2[1], 5)


class HomeZ2State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = Jog00State
        self.motor_index = 2


class Jog00State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = ManualState
        self.set_jog_target(0, 0, 5)


class PrintState(JogState):
    def __init__(self):
        super().__init__()
        self.start_time = time()

    def run(self):
        super().run()
        global errorx, errory
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
        control_inputx = KP*errorx + KP_VELOCITY*v_errorx
        
        errory = y_nominal - self.ypos
        control_inputy = KP*errory + KP_VELOCITY*v_errory

        control3, control4 = corexy_transform(control_inputx, control_inputy)

        # errorz2 = z_nominal - embedded_motors[2].theta
        # control_inputz2 = KP*errorz2 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[2].omega)
        # errorz1 = z_nominal - embedded_motors[1].theta
        # control_inputz1 = KP*errorz1 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[1].omega)
        # errorz0 = z_nominal - embedded_motors[0].theta
        # control_inputz0 = KP*errorz0 + KP_VELOCITY*(z_velocity_nominal - embedded_motors[0].omega)

        control_packet = pkt.pack_HeaderPacket(
            command=pkt.SerialCommand.RUN_MOTOR, motorCount=5)
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
    Z_JOG = 30
    XY_JOG = 20

    def __init__(self):
        super().__init__()
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=5)
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
            z_nominal *= ManualState.Z_JOG
            x_nominal = controller.axis_l.x
            if abs(x_nominal) < .2:
                x_nominal = 0
            x_nominal *= ManualState.XY_JOG
            y_nominal = controller.axis_l.y
            if abs(y_nominal) < .2:
                y_nominal = 0
            y_nominal *= ManualState.XY_JOG
        else:
            z_nominal = 0
            y_nominal = 0
            x_nominal = 0

        motor_3_control, motor_4_control = corexy_transform(x_nominal, y_nominal)
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=5)
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
    global state
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


if __name__ == "__main__":
    os.system(f"taskset -p -c 3 {os.getpid()}")
    cold_start(sys.argv[1])
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        start_time = time()
        jog_controller = controller
        # tracking_thread = Thread(target=run_tracking_loop, daemon=True)
        # tracking_thread.start()
        # print("Started tracking")
        sleep(2)
        embedded_thread = Thread(target=embedded_service, daemon=True)
        embedded_thread.start()
        print("Started controls")
        while True:
            sleep(1)
            # print(embedded_motors[4].theta, embedded_motors[3].theta, errorx, errory)
            # print(-100/XY_MM_PER_RAD, embedded_motors[3].theta, home_y)
            # print((-100/XY_MM_PER_RAD + home_y) - embedded_motors[3].theta)
            # print(state)
            # pos_error = math.sqrt(errorx**2 + errory**2)*XY_MM_PER_RAD
            # vel_error = math.sqrt(v_errorx**2 + v_errory**2)*XY_MM_PER_RAD
            # print(str(pos_error).ljust(30, ' '))
            # print(str(pos_error))
            # pos, vel = path_planner.get_solution(time()-start_time)
            # print(pos[0], vel)
            print(embedded_motors, state, jog_controller.button_a.is_pressed)
            # print(get_laser_displacement(), get_error(), state, controller.trigger_l.value)
            # print(corexy_inverse(embedded_motors[3].theta - FineHomeState.home_3, embedded_motors[4].theta- FineHomeState.home_4))
