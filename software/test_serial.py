import tkinter
from serial_host import packet_definitions as pkt
from threading import Thread
import numpy as np
from time import time
import os
import hid

# pid = 1158  #16c0
# vid = 5824  #0486

# device = hid.Device(vid, pid)
flags = os.O_RDWR
device = os.open('/dev/hidraw0', flags)

MAX_ACCELERATION = 500

nominal_theta = 0
nominal_theta2 = 0

embedded_motors = {}
embedded_sensors = {}

def transmit_packet(packet):
    os.write(device, b'\x00'+packet)
    # device.write(b'\x00'+packet)
    
def handleSlider(value):
    global nominal_theta
    nominal_theta = float(value)

def handleSlider2(value):
    global nominal_theta2
    nominal_theta2 = float(value)

def embedded_service():
    packet_count = 0
    KP = 20
    KD = 5
    KI = 0
    while True:
        # hid_msg = device.read(64, timeout=1000)
        hid_msg = os.read(device, 64)
        packet_count += 1
        # print(hid_msg)
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

        errorx = nominal_theta + nominal_theta2 - embedded_motors[0].theta
        control_inputx = np.clip(KP*errorx - KD*embedded_motors[0].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
        errory = nominal_theta - nominal_theta2 - embedded_motors[1].theta
        control_inputy = np.clip(KP*errory - KD*embedded_motors[1].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
        control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, motorCount=2)
        control_packet += pkt.pack_MotorPacket(embedded_motors[0].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputx)
        control_packet += pkt.pack_MotorPacket(embedded_motors[1].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputy)
        if packet_count >= 4:
            transmit_packet(control_packet)
            packet_count = 0

if __name__ == "__main__":
    os.system(f"taskset -p -c 3 {os.getpid()}")
    # embedded_service()
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
