import tkinter
from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from threading import Thread
import numpy as np
from time import time
import os
from xbox360controller import Xbox360Controller

CONTROLLER_DEAD_ZONE = 0.2
CONTROLLER_JOG_RATE = 50
MAX_ACCELERATION = 1000

x_nominal = 0
y_nominal = 0
z_nominal = 0

embedded_motors = {}
embedded_sensors = {}
jog_controller = None

def handleSlider(value):
    global x_nominal
    x_nominal = float(value)


def handleSlider2(value):
    global y_nominal
    y_nominal = float(value)


def embedded_service():
    packet_count = 0
    KP = 100
    KD = 0
    KI = 0
    while True:
        hid_msg = read()
        header = pkt.unpack_HeaderPacket(hid_msg[:pkt.size_HeaderPacket])
        unpack_index = pkt.size_HeaderPacket
        for i in range(header.motorCount):
            motor_packet = pkt.unpack_MotorPacket(
                hid_msg[unpack_index:unpack_index+pkt.size_MotorPacket])
            embedded_motors[motor_packet.motorId] = motor_packet
            unpack_index += pkt.size_MotorPacket
        for i in range(header.sensorCount):
            sensor_packet = pkt.unpack_SensorPacket(
                hid_msg[unpack_index:unpack_index+pkt.size_SensorPacket])
            embedded_sensors[sensor_packet.sensorId] = sensor_packet
            unpack_index += pkt.size_SensorPacket

        if jog_controller.button_trigger_l.is_pressed:
            if jog_controller.axis_l.x > CONTROLLER_DEAD_ZONE:
                x_nominal = (jog_controller.axis_l.x-CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            elif jog_controller.axis_l.x < -CONTROLLER_DEAD_ZONE:
                x_nominal = (jog_controller.axis_l.x+CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            else:
                x_nominal = 0

            if jog_controller.axis_l.y > CONTROLLER_DEAD_ZONE:
                y_nominal = (-jog_controller.axis_l.y+CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            elif jog_controller.axis_l.y < -CONTROLLER_DEAD_ZONE:
                y_nominal = (-jog_controller.axis_l.y-CONTROLLER_DEAD_ZONE)*CONTROLLER_JOG_RATE
            else:
                y_nominal = 0
            
            z_nominal = (jog_controller.trigger_l.value - jog_controller.trigger_r.value)*CONTROLLER_JOG_RATE
            
        else:
            x_nominal = 0
            y_nominal = 0
            z_nominal = 0
        errorx = x_nominal + y_nominal - embedded_motors[0].omega
        control_inputx = np.clip(
            KP*errorx - KD*embedded_motors[0].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
        control_inputx = np.clip(
            KP*errorx - KD*embedded_motors[0].omega, -MAX_ACCELERATION, MAX_ACCELERATION)

        errory = x_nominal - y_nominal - embedded_motors[1].omega
        control_inputy = np.clip(
            KP*errory - KD*embedded_motors[1].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
        errorz = z_nominal - embedded_motors[2].omega

        control_inputz = KP*errorz

        control_packet = pkt.pack_HeaderPacket(
            pkt.SerialCommand.RUN_MOTOR, motorCount=3)
        control_packet += pkt.pack_MotorPacket(
            embedded_motors[0].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputx)
        control_packet += pkt.pack_MotorPacket(
            embedded_motors[1].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputy)
        control_packet += pkt.pack_MotorPacket(
            embedded_motors[2].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputz)
        write(control_packet)


if __name__ == "__main__":
    os.system(f"taskset -p -c 3 {os.getpid()}")
    cold_start('/dev/hidraw0')
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        jog_controller = controller
        embedded_thread = Thread(target=embedded_service, daemon=True)
        embedded_thread.start()

        master = tkinter.Tk()
        w = tkinter.Scale(master, from_=40, to=-40, command=handleSlider,
                        variable=tkinter.DoubleVar(), width=40, length=200, resolution=.1)
        w.pack(side=tkinter.LEFT)
        w2 = tkinter.Scale(master, from_=40, to=-40, command=handleSlider2,
                        variable=tkinter.DoubleVar(), width=40, length=200, resolution=.1)
        w2.pack(side=tkinter.RIGHT)
        tkinter.mainloop()
