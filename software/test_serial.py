import serial
from serial_host import packet_definitions as pkt
serial_port = serial.Serial('COM4')  # open serial port
serial_port.flushInput()
serial_port.write(b'hello\n')     # write a string

input_str = b''

msg_length = None
while True:
    if serial_port.in_waiting:
        input_char = serial_port.read()
        input_str += input_char
        if len(input_str) == pkt.size_HeaderPacket:
            header = pkt.unpack_HeaderPacket(input_str)
            msg_length = header.length
            print(header)

        if msg_length and msg_length == len(input_str):
            break

unpack_index = pkt.size_HeaderPacket
for i in range(header.motorCount):
    print(pkt.unpack_MotorPacket(input_str[unpack_index:unpack_index+pkt.size_MotorPacket]))
    unpack_index += pkt.size_MotorPacket
for i in range(header.sensorCount):
    print(pkt.unpack_SensorPacket(input_str[unpack_index:unpack_index+pkt.size_SensorPacket]))
    unpack_index += pkt.size_SensorPacket
footer = pkt.unpack_FooterPacket(input_str[unpack_index:unpack_index+pkt.size_FooterPacket])
serial_port.close()             # close port
