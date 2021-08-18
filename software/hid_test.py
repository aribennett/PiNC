import hid
from serial_host import packet_definitions as pkt
from time import time
pid = 1158
vid = 5824

with hid.Device(vid, pid) as h:
    print(f'Device manufacturer: {h.manufacturer}')
    print(f'Product: {h.product}')
    print(f'Serial Number: {h.serial}')

device = hid.Device(vid, pid)
embedded_motors = {}
embedded_sensors = {}

print_time = time()
packet_count = 0


hid_msg = device.read(64, timeout=1000)
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

control_packet = pkt.pack_HeaderPacket(command=pkt.SerialCommand.RUN_MOTOR, motorCount=1)
control_packet += pkt.pack_MotorPacket(embedded_motors[0].motorId, pkt.MotorCommand.SET_OMEGA, omega=10)
print(control_packet)
device.write(b'\x00'+control_packet) # add extra byte for report descriptor. I think it gets eaten
# while True:
#     hid_msg = device.read(64, timeout=1000)
#     header = pkt.unpack_HeaderPacket(hid_msg[:pkt.size_HeaderPacket])
#     unpack_index = pkt.size_HeaderPacket
#     for i in range(header.motorCount):
#         motor_packet = pkt.unpack_MotorPacket(
#             hid_msg[unpack_index:unpack_index+pkt.size_MotorPacket])
#         embedded_motors[motor_packet.motorId] = motor_packet
#         unpack_index += pkt.size_MotorPacket
#     for i in range(header.sensorCount):
#         sensor_packet = pkt.unpack_SensorPacket(
#             hid_msg[unpack_index:unpack_index+pkt.size_SensorPacket])
#         embedded_sensors[sensor_packet.sensorId] = sensor_packet
#         unpack_index += pkt.size_SensorPacket
#     packet_count += 1
#     print(embedded_motors)
packet_count += 1
