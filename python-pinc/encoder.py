import RPi.GPIO as GPIO
import time
import os

os.system('taskset -p -c 3 %d' % os.getpid())

ENC_A = 11
ENC_B = 13

count = 0

def handle_enc_a(pin):
    global count
    state = GPIO.input(ENC_A)
    dir = GPIO.input(ENC_B)
    if state:
        if dir:
            count += 1
        else:
            count -= 1
    else:
        if dir:
            count -= 1
        else:
            count += 1


GPIO.setmode(GPIO.BOARD)
GPIO.setup([ENC_A, ENC_B], GPIO.IN)
GPIO.add_event_detect(ENC_A, GPIO.BOTH, handle_enc_a)

def get_encoder_value():
    return count

# while True:
#     print(int(count/2))
#     time.sleep(.1)