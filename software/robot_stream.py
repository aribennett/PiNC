from serial_host.robot_interface import RobotInterface

main = RobotInterface()
while True:
    main.run()
    print(main.sensors)