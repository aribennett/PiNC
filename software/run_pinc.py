from serial_host import packet_definitions as pkt
from serial_host.robot_interface import RobotInterface
from path_planning.gcode_solver import GcodeSolver
from kinematics import corexy_inverse, corexy_transform, z0, z1, z2, center_home
from laser_tracking import get_laser_displacement, run_tracking_loop, end_tracking_loop
from threading import Thread
import numpy as np
from time import time, sleep
from queue import Queue
import os
from xbox360controller import Xbox360Controller
from pinc_state import State
import sys
import logging
from thermistor import get_thermistor_temp

XY_MM_PER_RAD = 6.36619783227
Z_MM_PER_RAD = 0.795774715
E_MM_PER_RAD = .5
FINE_Z = 14

# ------ Debug Variables --------
errorx = 0
errory = 0
# -------------------------------

jog_controller = None

with open('111cube.gcode', 'r') as f:
    gcode = f.read()

path_planner = GcodeSolver(gcode, start_position=[0, 0, 0])

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
        # self.event_map['init'] = ManualState

    def run(self):
        post_event('init')


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
        main.send_command()

    def set_jog_target(self, x, y, z, time):
        self.jog_time = time
        self.x_target, self.y_target, self.z_target = x, y, z
        self.xw_nominal = (self.x_target - self.xstart)/self.jog_time
        self.yw_nominal = (self.y_target - self.ystart)/self.jog_time
        self.zw_nominal = (self.z_target - self.zstart)/self.jog_time
        print("Znominal", self.zw_nominal)

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
            main.send_command()
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
            main.send_command()


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
        # self.event_map['found home'] = HeatState

        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.ENABLE))
        main.add_output_command(pkt.pack_ComponentPacket(0, 1))
        main.add_output_command(pkt.pack_ComponentPacket(1, 1))
        main.add_output_command(pkt.pack_ComponentPacket(2, 0))
        main.send_command()
        self.last_home = main.get_motor_state(3)[0]
        self.last_timeout = -1

    def run(self):
        # init time on first run
        if self.last_timeout == -1:
            self.last_timeout = time() + 0.1

        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=HomeState.HOMING_SPEED))
        main.send_command()

        if main.get_motor_state(3)[0] - self.last_home > HomeState.HOME_THRESHHOLD:
            self.last_timeout = time()
            self.last_home = main.get_motor_state(3)[0]
        elif time() - self.last_timeout > HomeState.HOME_TIMEOUT:
            HomeState.home_0 = main.get_motor_state(0)[0]
            HomeState.home_1 = main.get_motor_state(1)[0]
            HomeState.home_2 = main.get_motor_state(2)[0]
            HomeState.home_3 = main.get_motor_state(3)[0]
            HomeState.home_4 = main.get_motor_state(4)[0]
            main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=0))
            main.send_command()
            post_event('found home')


class HomeZState(State):
    def __init__(self):
        super().__init__()
        self.motor_index = 'all'

    def run(self):
        z_nominal = np.clip(-get_laser_displacement()/10, -10, 10)
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=0))
        main.send_command()

        if z_nominal == 0:
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
        self.set_jog_target(30, 30, -20, 5)


class HomeCenterState(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome0State


class JogHome0State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ0State
        self.set_jog_target(z0[0], z0[1], -10, 5)


class HomeZ0State(HomeZState):
    def __init__(self):
        super().__init__()
        self.motor_index = 0
        self.event_map['z home'] = JogHome1State


class JogHome1State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ1State
        self.set_jog_target(z1[0], z1[1], -10, 5)


class HomeZ1State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = JogHome2State
        self.motor_index = 1


class JogHome2State(JogState):
    def __init__(self):
        super().__init__()
        self.event_map['jog done'] = HomeZ2State
        self.set_jog_target(z2[0], z2[1], -10, 5)


class HomeZ2State(HomeZState):
    def __init__(self):
        super().__init__()
        self.event_map['z home'] = Jog00State
        self.motor_index = 2


class Jog00State(JogState):
    def __init__(self):
        super().__init__()
        HomeState.home_0 += FINE_Z/Z_MM_PER_RAD
        HomeState.home_1 += FINE_Z/Z_MM_PER_RAD
        HomeState.home_2 += FINE_Z/Z_MM_PER_RAD
        self.event_map['jog done'] = HeatState
        self.set_jog_target(0, 0, 0, 5)


class HeatState(State):
    def __init__(self):
        super().__init__()
        end_tracking_loop()
        self.event_map['done heating'] = PrintState

    def run(self):
        super().run()
        temp_error = ManualState.NOMINAL_TEMP - get_thermistor_temp(main.sensors[0].value)[0]
        control = int(np.clip(temp_error*100, 0, 4096))

        main.add_output_command(pkt.pack_ComponentPacket(4, control))
        main.send_command()

        if abs(temp_error) < 15:
            post_event('done heating')


class PrintState(State):
    def __init__(self):
        super().__init__()
        self.start_time = time()
        NOMINAL_TEMP = 240
        self.home_e = main.get_motor_state(5)[0]
        main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.ENABLE))
        main.add_output_command(pkt.pack_ComponentPacket(0, 0))
        main.send_command()

    def run(self):
        super().run()
        self.xpos, self.ypos = corexy_inverse(main.get_motor_state(3)[0] - HomeState.home_3, main.get_motor_state(4)[0] - HomeState.home_4)
        self.xvel, self.yvel = corexy_inverse(main.get_motor_state(3)[1], main.get_motor_state(4)[1])
        self.z0pos = main.get_motor_state(0)[0] - HomeState.home_0
        self.z1pos = main.get_motor_state(1)[0] - HomeState.home_1
        self.z2pos = main.get_motor_state(2)[0] - HomeState.home_2
        self.epos = main.get_motor_state(5)[0] - self.home_e
        global errorx, errory
        KP = 50
        KP_VELOCITY = 0
        positions, velocities = path_planner.get_solution(time()-self.start_time)
        position = positions[0]
        x_nominal = position[0]/XY_MM_PER_RAD
        y_nominal = position[1]/XY_MM_PER_RAD
        z_nominal = -position[2]/Z_MM_PER_RAD
        e_nominal = position[3]/E_MM_PER_RAD

        x_velocity_nominal = velocities[0]/XY_MM_PER_RAD
        y_velocity_nominal = velocities[1]/XY_MM_PER_RAD
        z_velocity_nominal = velocities[2]/Z_MM_PER_RAD
        v_errorx = x_velocity_nominal - self.xvel
        v_errory = y_velocity_nominal - self.yvel

        errorx = x_nominal - self.xpos
        control_inputx = KP*errorx + KP_VELOCITY*v_errorx

        errory = y_nominal - self.ypos
        control_inputy = KP*errory + KP_VELOCITY*v_errory

        error_e = e_nominal-self.epos
        control_inpute = error_e*10

        control3, control4 = corexy_transform(control_inputx, control_inputy)

        errorz2 = z_nominal - self.z2pos
        control_inputz2 = KP*errorz2
        errorz1 = z_nominal - self.z1pos
        control_inputz1 = KP*errorz1
        errorz0 = z_nominal - self.z0pos
        control_inputz0 = KP*errorz0
        temp_error = ManualState.NOMINAL_TEMP - get_thermistor_temp(main.sensors[0].value)[0]
        control = int(np.clip(temp_error*1000, 0, 4096))

        main.add_output_command(pkt.pack_ComponentPacket(4, control))
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=control_inputz0))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=control_inputz1))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=control_inputz2))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=control3))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=control4))
        main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.SET_OMEGA, control=control_inpute))
        main.send_command()


class ManualState(State):
    Z_JOG = 30
    XY_JOG = 30
    XY_P_ACCEL = 40
    NOMINAL_TEMP = 215

    def __init__(self):
        super().__init__()
        end_tracking_loop()
        main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.ENABLE))
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.ENABLE))
        main.add_output_command(pkt.pack_ComponentPacket(0, 0))
        main.send_command()

    def run(self):
        if jog_controller.button_a.is_pressed:
            z_nominal = (controller.trigger_l.value - controller.trigger_r.value)
            if abs(z_nominal) < .1:
                z_nominal = 0
            z_nominal *= ManualState.Z_JOG
            x_nominal = controller.axis_l.x
            if abs(x_nominal) < .1:
                x_nominal = 0
            else:
                if x_nominal > 0:
                    x_nominal -= .1
                else:
                    x_nominal += .1

            x_nominal *= ManualState.XY_JOG
            y_nominal = controller.axis_l.y
            if abs(y_nominal) < .1:
                y_nominal = 0
            else:
                if y_nominal > 0:
                    y_nominal -= .1
                else:
                    y_nominal += .1

            y_nominal *= ManualState.XY_JOG
        else:
            z_nominal = 0
            y_nominal = 0
            x_nominal = 0

        temp_error = ManualState.NOMINAL_TEMP - get_thermistor_temp(main.sensors[0].value)[0]
        control = int(np.clip(temp_error*100, 0, 4096))

        main.add_output_command(pkt.pack_ComponentPacket(4, control))

        motor_3_control, motor_4_control = corexy_transform(x_nominal, y_nominal)
        motor_3_error = (motor_3_control - main.get_motor_state(3)[1]) * ManualState.XY_P_ACCEL
        motor_4_error = (motor_4_control - main.get_motor_state(4)[1]) * ManualState.XY_P_ACCEL

        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=z_nominal))
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=z_nominal))

        if jog_controller.button_a.is_pressed or main.get_motor_state(3)[1] != 0 or main.get_motor_state(4)[1] != 0:
            main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_ALPHA, control=motor_3_error))
            main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_ALPHA, control=motor_4_error))
        else:
            # handle the int floor case for velocity
            main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_OMEGA, control=0))
            main.add_motor_command(pkt.pack_MotorCommandPacket(4, pkt.MotorCommand.SET_OMEGA, control=0))

        if jog_controller.button_x.is_pressed:
            main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.SET_OMEGA, control=10))
        elif jog_controller.button_y.is_pressed:
            main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.SET_OMEGA, control=-10))
        else:
            main.add_motor_command(pkt.pack_MotorCommandPacket(5, pkt.MotorCommand.SET_OMEGA, control=0))

        main.send_command()


def embedded_service():
    global state
    state = InitState()
    while True:
        main.run()
        handle_events()
        state.run()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    os.system(f"taskset -p -c 3 {os.getpid()}")
    main = RobotInterface()
    # with Xbox360Controller(0, axis_threshold=0.2) as controller:
    start_time = time()
    # jog_controller = controller
    tracking_thread = Thread(target=run_tracking_loop, daemon=True)
    tracking_thread.start()
    print("Started tracking")
    sleep(2)
    embedded_thread = Thread(target=embedded_service, daemon=True)
    embedded_thread.start()
    print("Started controls")
    while True:
        sleep(.1)
        print(errorx*XY_MM_PER_RAD, errory*XY_MM_PER_RAD, get_thermistor_temp(main.sensors[0].value))
        # print(get_thermistor_temp(main.sensors[0].value))
            # print(get_laser_displacement())
            # print(HomeState.home_0, HomeState.home_1, HomeState.home_2)
