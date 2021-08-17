import serial
import tkinter
from serial import SerialTimeoutException
import serial.tools.list_ports
from serial_host import packet_definitions as pkt
from time import sleep, time_ns, time
from threading import Thread
import numpy as np
pid = "0483"
hid = "16C0"
SERIAL_TIMEOUT = .005
MAX_ACCELERATION = 500
SERIAL_FRAME_RATE = 2000
SERIAL_INTERVAL = 1/SERIAL_FRAME_RATE
print(SERIAL_INTERVAL)
serial_device = None

packet_count = 0

nominal_theta = 0
nominal_theta2 = 0

embedded_motors = {}
embedded_sensors = {}

def transmit_packet(packet):
    global packet_count
    serial_port.reset_input_buffer()
    serial_port.write(packet)     # write a string
    # print(serial_port.read_until())
    input_str = b''

    current_size = pkt.size_HeaderPacket
    input_str += serial_port.read(pkt.size_HeaderPacket)
    if current_size != len(input_str):
        raise SerialTimeoutException

    header = pkt.unpack_HeaderPacket(input_str)
    current_size = header.length
    input_str += serial_port.read(header.length - pkt.size_HeaderPacket)
    if current_size != len(input_str):
        raise SerialTimeoutException

    unpack_index = pkt.size_HeaderPacket
    for i in range(header.motorCount):
        motor_packet = pkt.unpack_MotorPacket(input_str[unpack_index:unpack_index+pkt.size_MotorPacket])
        embedded_motors[motor_packet.motorDescriptor.decode("utf-8").split("\x00")[0]] = motor_packet
        unpack_index += pkt.size_MotorPacket
    for i in range(header.sensorCount):
        sensor_packet = pkt.unpack_SensorPacket(input_str[unpack_index:unpack_index+pkt.size_SensorPacket])
        embedded_sensors[sensor_packet.sensorDescriptor.decode("utf-8").split("\x00")[0]] = sensor_packet
        unpack_index += pkt.size_SensorPacket
    footer = pkt.unpack_FooterPacket(
        input_str[unpack_index:unpack_index+pkt.size_FooterPacket])
    
    packet_count += 1

def initialize_device():
    status_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.GET_STATUS, pkt.size_HeaderPacket+pkt.size_FooterPacket) + pkt.pack_FooterPacket()
    transmit_packet(status_packet)

def handleSlider(value):
    global nominal_theta
    nominal_theta = float(value)

def handleSlider2(value):
    global nominal_theta2
    nominal_theta2 = float(value)

def embedded_service():
    global packet_count
    start_time = time()
    print_time = time()
    control_time = time_ns()
    while True:
        if time() - print_time > 1:
            print_time = time()
            print(packet_count)
            packet_count = 0
        try:
            KP = 100
            KD = 20
            KI = 0
            errorx = nominal_theta + nominal_theta2 - embedded_motors['xy1'].theta
            control_inputx = np.clip(KP*errorx - KD*embedded_motors['xy1'].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
            errory = nominal_theta - nominal_theta2 - embedded_motors['xy2'].theta
            control_inputy = np.clip(KP*errory - KD*embedded_motors['xy2'].omega, -MAX_ACCELERATION, MAX_ACCELERATION)
            control_packet = pkt.pack_HeaderPacket(pkt.SerialCommand.RUN_MOTOR, pkt.size_HeaderPacket + 2*pkt.size_MotorPacket + pkt.size_FooterPacket, motorCount=2)
            control_packet += pkt.pack_MotorPacket(embedded_motors['xy1'].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputx)
            control_packet += pkt.pack_MotorPacket(embedded_motors['xy2'].motorId, pkt.MotorCommand.SET_ALPHA, alpha=control_inputy)
            control_packet += pkt.pack_FooterPacket()
            transmit_packet(control_packet)
        except:
            print('failed after', packet_count, time()-start_time)
            break
        sleep(SERIAL_INTERVAL)
        # print(nominal_theta - embedded_motors['xy1'].theta)

if __name__ == "__main__":
    for port in serial.tools.list_ports.comports():
        if pid and hid in port.hwid:
            serial_device = port.device
            break

    if not serial_device:
        print("No serial device found.")
        exit()

    serial_port = serial.Serial(
        serial_device, timeout=SERIAL_TIMEOUT)  # open serial port

    initialize_device()
    embedded_service()
    # embedded_thread = Thread(target=embedded_service, daemon=True)
    # embedded_thread.start()

    # master = tkinter.Tk()
    # w = tkinter.Scale(master, from_=40, to=-40, command=handleSlider,
    #                 variable=tkinter.DoubleVar(), width=40, length=200, resolution=.1)
    # w.pack(side=tkinter.LEFT)
    # w2 = tkinter.Scale(master, from_=40, to=-40, command=handleSlider2,
    #                 variable=tkinter.DoubleVar(), width=40, length=200, resolution=.1)
    # w2.pack(side=tkinter.RIGHT)
    # tkinter.mainloop()
