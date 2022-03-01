from . import cold_start, read, write, close_device
from . import packet_definitions as pkt


class RobotInterface(object):
    def __init__(self):
        self.motors = {}
        self.sensors = {}
        self.motor_command_queue = b''
        self.motor_command_count = 0
        cold_start()

    def run(self):
        hid_msg = read()
        header = pkt.unpack_HeaderPacket(hid_msg[:pkt.size_HeaderPacket])
        unpack_index = pkt.size_HeaderPacket
        for i in range(header.motorCount):
            motor_packet = pkt.unpack_MotorStatePacket(hid_msg[unpack_index:unpack_index+pkt.size_MotorStatePacket])
            self.motors[motor_packet.motorId] = motor_packet
            unpack_index += pkt.size_MotorStatePacket
        for i in range(header.componentCount):
            sensor_packet = pkt.unpack_ComponentPacket(hid_msg[unpack_index:unpack_index+pkt.size_ComponentPacket])
            self.sensors[sensor_packet.componentId] = sensor_packet
            unpack_index += pkt.size_ComponentPacket

    def get_motor_state(self, index=0):
        return self.motors[index].theta/250, self.motors[index].omega/100, self.motors[index].alpha/100

    def add_motor_command(self, command):
        self.motor_command_queue += command
        self.motor_command_count += 1

    def send_command(self):
        control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN, motorCount=self.motor_command_count)
        control_packet += self.motor_command_queue
        write(control_packet)
        self.motor_command_queue = b''
        self.motor_command_count = 0
