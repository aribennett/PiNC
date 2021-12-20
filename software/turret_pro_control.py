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
    main = RobotInterface(0x16c0, 0x0486)
    robot = threading.Thread(target=robot_thread, daemon=True)
    robot.start()
    gamepad = hid.Device(0x057e, 0x2009)
    while True:
        read_byte = gamepad.read(64, 1000)
        dpad = read_byte[5]
        button_pad = read_byte[3]
        up_down = (read_byte[8] & 0xFC)/4 - 32
        left_right = (read_byte[7] & 0x0f) * 4 - 32
        a = button_pad & 8
        zr = button_pad & 128

        x_vel = 0
        y_vel = 0
        pwm = 0
        angle = 0

        if abs(left_right) > DEADBAND:
            x_vel = left_right/4
        if abs(up_down) > DEADBAND:
            y_vel = up_down/4
        if a:
            pwm = 45
        if a and zr:
            angle = 0
        else:
            angle = 90
        print(up_down, left_right)
