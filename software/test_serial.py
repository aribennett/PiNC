import serial
from serial.serialwin32 import Serial

serial_port = serial.Serial('COM4')  # open serial port
serial_port.write(b'hello\n')     # write a string

input_str = b''

while True:
    if serial_port.in_waiting:
        input_char = serial_port.read()
        input_str += input_char
        if input_char == b'\n':
            break

output_str = ""
for byte in input_str:
    output_str += str(int(byte)) + ', '
print("Message Length: ", len(input_str) - 2)
print(output_str)

serial_port.close()             # close port
