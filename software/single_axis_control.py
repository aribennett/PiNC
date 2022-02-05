from time import sleep
from serial_host.robot_interface import RobotInterface
from serial_host import packet_definitions as pkt
import threading
import hid

DEADBAND = 5

x_vel = 0
y_vel = 0
pwm = 0
angle = 90


def robot_thread():
    main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.ENABLE))
    main.send_command()
    while True:
        main.run()
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=y_vel))
        main.send_command()


if __name__ == "__main__":
    main = RobotInterface(5824, 1158)
    robot = threading.Thread(target=robot_thread, daemon=True)
    robot.start()
    gamepad = hid.Device(0x057e, 0x2009)
    while True:
        sleep(1)
        read_byte = gamepad.read(64, 1000)
        up_down = (read_byte[8] & 0xFC)/4 - 32
        y_vel = 0
        if abs(up_down) > DEADBAND:
            y_vel = up_down/4
        print(main.get_motor_state(0))
