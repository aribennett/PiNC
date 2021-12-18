from serial_host.robot_interface import RobotInterface
from serial_host import packet_definitions as pkt

from time import time

main = RobotInterface()

last_time = time()

while True:
    main.run()
    main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_ALPHA, control=1))
    main.send_command()
    if time() - last_time > .1:
        last_time = time()
        print(main.motors)
