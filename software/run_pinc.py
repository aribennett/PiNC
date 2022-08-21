from serial_host import packet_definitions as pkt
from serial_host.robot_interface import RobotInterface
from path_planning.gcode_solver import GcodeSolver
from kinematics import corexy_inverse, corexy_transform, z0, z1, z2, center_home
from threading import Thread
import numpy as np
from time import time, sleep
from queue import Queue
import os
from pinc_state import State
import logging
from thermistor import get_thermistor_temp, get_ntc100k_temp

os.chdir(os.path.dirname(__file__))

XY_MM_PER_RAD = 6.36619783227
Z_MM_PER_RAD = 0.63661977236
E_MM_PER_RAD = .73
FINE_Z = 1.3
PRESSURE_ADVANCE = 1.005

# ------ Debug Variables --------
errorx = 0
errory = 0
v_errorx = 0
v_errory = 0
# -------------------------------

jog_controller = None

with open('gcode_examples/sw.gcode', 'r') as f:
    gcode = f.read()

path_planner = GcodeSolver(gcode, start_position=[-XY_MM_PER_RAD, -XY_MM_PER_RAD, 0])

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
        main.send_command()


class InitState(State):
    def __init__(self):
        super().__init__()
        self.event_map['init'] = HeatState

    def run(self):
        post_event('init')


class IdleState(State):
    def __init__(self):
        super().__init__()


class JogState(State):
    def __init__(self):
        super().__init__()
        self.xstart, self.ystart = corexy_inverse(main.get_motor_state(3)[0] - HomeState.home_3, main.get_motor_state(4)[0] - HomeState.home_4)
        self.zstart = main.get_motor_state(0)[0] - HomeState.home_0
        self.jog_time = 10
        self.start_time = time()
        self.x_target, self.y_target, self.z_target = 0, 0, 0
        self.xw_nominal, self.yw_nominal = 0, 0
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.ENABLE))

    def set_jog_target(self, x, y, z, time):
        self.jog_time = time
        self.x_target, self.y_target, self.z_target = x, y, z
        self.xw_nominal = (self.x_target - self.xstart)/self.jog_time
        self.yw_nominal = (self.y_target - self.ystart)/self.jog_time
        self.zw_nominal = (self.z_target - self.zstart)/self.jog_time

    def run(self):
        self.xpos, self.ypos = corexy_inverse(main.get_motor_state(3)[0] - HomeState.home_3, main.get_motor_state(4)[0] - HomeState.home_4)
        self.xvel, self.yvel = corexy_inverse(main.get_motor_state(3)[1], main.get_motor_state(4)[1])
        self.z0pos = main.get_motor_state(0)[0] - HomeState.home_0
        self.z1pos = main.get_motor_state(1)[0] - HomeState.home_1
        self.z2pos = main.get_motor_state(2)[0] - HomeState.home_2

        interp = (time()-self.start_time)/self.jog_time
        if interp >= 1:
            main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=0))
            main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=0))
            main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=0))
            main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=0))
            main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=0))
            post_event('jog done')
        else:
            x_nominal = interp*(self.x_target - self.xstart) + self.xstart
            y_nominal = interp*(self.y_target - self.ystart) + self.ystart
            z_nominal = interp*(self.z_target - self.zstart) + self.zstart

            x_error = x_nominal - self.xpos
            y_error = y_nominal - self.ypos
            z0_error = z_nominal - self.z0pos
            z1_error = z_nominal - self.z1pos
            z2_error = z_nominal - self.z2pos

            control_x = x_error + self.xw_nominal
            control_y = y_error + self.yw_nominal

            control_3, control_4 = corexy_transform(control_x, control_y)

            control_0 = self.zw_nominal + z0_error
            control_1 = self.zw_nominal + z1_error
            control_2 = self.zw_nominal + z2_error

            main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=control_0))
            main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=control_1))
            main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=control_2))
            main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=control_3))
            main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=control_4))


class HomeState(State):
    HOMING_SPEED = 3
    HOME_TIMEOUT = .05
    HOME_THRESHHOLD = .05

    home_0 = 0
    home_1 = 0
    home_2 = 0
    home_3 = 0
    home_4 = 0

    def __init__(self):
        super().__init__()
        self.event_map['found home'] = JogHomeCenterState

        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.ENABLE))
        main.add_output_command(pkt.pack_OutputCommandPacket(0, 1))
        main.add_output_command(pkt.pack_OutputCommandPacket(1, 1))
        main.add_output_command(pkt.pack_OutputCommandPacket(2, 0))
        main.add_output_command(pkt.pack_OutputCommandPacket(3, 0))
        main.add_output_command(pkt.pack_OutputCommandPacket(4, 0))
        self.last_home = main.get_motor_state(4)[0]
        self.last_timeout = -1

    def run(self):
        # init time on first run
        if self.last_timeout == -1:
            self.last_timeout = time() + 0.1

        if abs(main.get_motor_state(4)[0] - self.last_home) > HomeState.HOME_THRESHHOLD:
            self.last_timeout = time()
            self.last_home = main.get_motor_state(4)[0]
        elif time() - self.last_timeout > HomeState.HOME_TIMEOUT:
            HomeState.home_0 = main.get_motor_state(0)[0]
            HomeState.home_1 = main.get_motor_state(1)[0]
            HomeState.home_2 = main.get_motor_state(2)[0]
            HomeState.home_3 = main.get_motor_state(3)[0]
            HomeState.home_4 = main.get_motor_state(4)[0]
            main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=0))
            post_event('found home')
            return

        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=-HomeState.HOMING_SPEED))


class HomeZState(State):
    def __init__(self):
        super().__init__()
        self.motor_index = 'all'
        self.z_nominal = 10

    def run(self):
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=self.z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=self.z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=self.z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=0))

        if main.sensors[2].value != 0:
            if self.motor_index == 0:
                HomeState.home_0 = main.get_motor_state(0)[0]
            elif self.motor_index == 1:
                HomeState.home_1 = main.get_motor_state(1)[0]
            elif self.motor_index == 2:
                HomeState.home_2 = main.get_motor_state(2)[0]
            else:
                HomeState.home_0 = main.get_motor_state(0)[0]
                HomeState.home_1 = main.get_motor_state(1)[0]
                HomeState.home_2 = main.get_motor_state(2)[0]
            post_event('z home')


class JogHomeCenterState(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeCenterState
        self.set_jog_target(center_home[0], center_home[1], -50, 2)


class HomeCenterState(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome0State


class JogHome0State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ0State
        self.set_jog_target(z0[0], z0[1], -10, 2)


class HomeZ0State(HomeZState):
    def __init__(self):
        super().__init__()
        self.motor_index = 0
        self.event_map['z home'] = JogHome1State
        self.z_nominal = 3


class JogHome1State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ1State
        self.set_jog_target(z1[0], z1[1], -10, 2)


class HomeZ1State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome2State
        self.motor_index = 1
        self.z_nominal = 3


class JogHome2State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ2State
        self.set_jog_target(z2[0], z2[1], -10, 2)


class HomeZ2State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = Jog00State
        self.motor_index = 2
        self.z_nominal = 3


class Jog00State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = JogOffsetState
        self.set_jog_target(-1, -1, 0, 5)


class JogOffsetState(JogState):
    def __init__(self):
        super().__init__()
        # self.event_map['jog done'] = IdleState
        self.event_map['jog done'] = PrintState
        self.set_jog_target(-1, -1, FINE_Z/Z_MM_PER_RAD, 5)


class HeatState(State):
    NOMINAL_TEMP = 225
    NOMINAL_TEMP_BED = 60

    def __init__(self):
        super().__init__()
        self.event_map['done heating'] = HomeState

    def run(self):
        super().run()
        temp_error = HeatState.NOMINAL_TEMP - get_thermistor_temp(main.sensors[0].value)[0]
        control = int(np.clip(temp_error*1000, 0, 4096))
        bed_control = int(get_thermistor_temp(main.sensors[1].value)[0] < HeatState.NOMINAL_TEMP_BED)

        main.add_output_command(pkt.pack_OutputCommandPacket(3, control))
        main.add_output_command(pkt.pack_OutputCommandPacket(4, bed_control))
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=0))

        if abs(temp_error) < 5:
            post_event('done heating')


class PrintState(State):
    def __init__(self):
        super().__init__()
        self.start_time = time()
        self.home_e = main.get_motor_state(5)[0]
        self.enabled_fan = False
        main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.ENABLE))
        main.add_output_command(pkt.pack_OutputCommandPacket(0, 0))

    def run(self):
        super().run()
        self.apos, self.bpos = main.get_motor_state(3)[0] - HomeState.home_3, main.get_motor_state(4)[0] - HomeState.home_4
        self.avel, self.bvel = main.get_motor_state(3)[1], main.get_motor_state(4)[1]
        self.z0pos = main.get_motor_state(0)[0] - HomeState.home_0
        self.z1pos = main.get_motor_state(1)[0] - HomeState.home_1
        self.z2pos = main.get_motor_state(2)[0] - HomeState.home_2
        self.epos = main.get_motor_state(5)[0] - self.home_e
        global errorx, errory, v_errorx, v_errory
        KP = 15
        KPZ = 50
        positions, velocities, cooling = path_planner.get_solution(time()-self.start_time)
        position = positions[0]
        x_nominal = position[0]/XY_MM_PER_RAD
        y_nominal = position[1]/XY_MM_PER_RAD
        a_nominal, b_nominal = corexy_transform(x_nominal, y_nominal)

        z_nominal = -position[2]/Z_MM_PER_RAD + FINE_Z/Z_MM_PER_RAD
        e_nominal = position[3]/E_MM_PER_RAD

        x_velocity_nominal = velocities[0]/XY_MM_PER_RAD
        y_velocity_nominal = velocities[1]/XY_MM_PER_RAD
        z_velocity_nominal = 0;#velocities[2]/Z_MM_PER_RAD # NEED TO FIX
        e_velocity_nominal = (velocities[3]/E_MM_PER_RAD)*PRESSURE_ADVANCE
        a_velocity_nominal, b_velocity_nominal = corexy_transform(x_velocity_nominal, y_velocity_nominal)

        v_errorx = a_velocity_nominal - self.avel
        v_errory = b_velocity_nominal - self.bvel

        errorx = a_nominal - self.apos
        control3 = KP * errorx + a_velocity_nominal

        errory = b_nominal - self.bpos
        control4 = KP * errory + b_velocity_nominal

        error_e = e_nominal-self.epos
        control_inpute = error_e*100 + e_velocity_nominal

        errorz2 = z_nominal - self.z2pos
        control_inputz2 = KPZ*errorz2 + z_velocity_nominal
        errorz1 = z_nominal - self.z1pos
        control_inputz1 = KPZ*errorz1 + z_velocity_nominal
        errorz0 = z_nominal - self.z0pos
        control_inputz0 = KPZ*errorz0 + z_velocity_nominal

        temp_error = HeatState.NOMINAL_TEMP - get_thermistor_temp(main.sensors[0].value)[0]
        control = int(np.clip(temp_error*1000, 0, 4096))
        bed_control = int(get_thermistor_temp(main.sensors[1].value)[0] < HeatState.NOMINAL_TEMP_BED)

        main.add_output_command(pkt.pack_OutputCommandPacket(3, control))
        main.add_output_command(pkt.pack_OutputCommandPacket(4, bed_control))
        main.add_output_command(pkt.pack_OutputCommandPacket(2, int(cooling*8)))
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=control_inputz0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=control_inputz1))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=control_inputz2))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=control3))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=control4))
        main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.SET_OMEGA, control=control_inpute))


def embedded_service():
    global state
    state = InitState()
    while True:
        main.run()
        handle_events()
        state.run()
        main.send_command()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    os.system(f"taskset -p -c 3 {os.getpid()}")
    main = RobotInterface()
    start_time = time()
    embedded_thread = Thread(target=embedded_service, daemon=True)
    embedded_thread.start()
    print("Started controls")
    while True:
        sleep(.1)
        print(f"{errorx*XY_MM_PER_RAD:9.4f}, {errory*XY_MM_PER_RAD:9.4f}, {v_errorx*XY_MM_PER_RAD:9.4f}, {v_errory*XY_MM_PER_RAD:9.4f}, {get_thermistor_temp(main.sensors[0].value)[0]:9.4f}", end='\r')
