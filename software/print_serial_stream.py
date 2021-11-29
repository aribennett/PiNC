from serial_host import packet_definitions as pkt
from serial_host import cold_start, read, write
from threading import Thread
from time import time, sleep
from queue import Queue
import logging

embedded_motors = {}
embedded_sensors = {}
packet_count = 0

def embedded_service():
    global packet_count
    while True:
        hid_msg = read()
        packet_count += 1
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

if __name__ == "__main__":
    cold_start()
    embedded_thread = Thread(target=embedded_service, daemon=True)
    embedded_thread.start()
    while True:
        sleep(1)
        print(embedded_motors)

