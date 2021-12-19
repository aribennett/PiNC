from serial_host.robot_interface import RobotInterface
from serial_host import packet_definitions as pkt
import hid
import threading
from time import time, sleep
import inputs


x_vel = 0
y_vel = 0
pwm = 0
angle = 90


def robot_thread():
    main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.ENABLE))
    main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.ENABLE))
    main.send_command()
    while True:
        main.run()
        main.add_motor_command(pkt.pack_MotorCommandPacket(0, pkt.MotorCommand.SET_OMEGA, control=y_vel))
        main.add_motor_command(pkt.pack_MotorCommandPacket(1, pkt.MotorCommand.SET_OMEGA, control=x_vel))
        main.add_motor_command(pkt.pack_MotorCommandPacket(2, pkt.MotorCommand.SET_OMEGA, control=pwm))
        main.add_motor_command(pkt.pack_MotorCommandPacket(3, pkt.MotorCommand.SET_THETA, control=angle))
        main.send_command()


if __name__ == "__main__":
    main = RobotInterface()
    robot = threading.Thread(target=robot_thread, daemon=True)
    robot.start()

    gamepad = hid.Device(0x057e, 0x2009)
    while True:
        read_byte = gamepad.read(64, 1000)
        dpad = read_byte[5]
        button_pad = read_byte[3]
        up = dpad & 2
        down = dpad & 1
        left = dpad & 8
        right = dpad & 4
        a = button_pad & 8
        zr = button_pad & 128

        x_vel = 0
        y_vel = 0
        pwm = 0
        angle = 0

        if left:
            x_vel -= 5
        if right:
            x_vel += 5
        if up:
            y_vel += 5
        if down:
            y_vel -= 5

        if a:
            pwm = 45

        if a and zr:
            angle = 0
        else:
            angle = 90