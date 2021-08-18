from time import time, time_ns, sleep
from arduino_timing import delayMicroseconds
start_time = time()
interval_time = time_ns()
count = 0
while True:
    delayMicroseconds(1000)
    count += 1
    interval_time = time_ns()
    if count == 1000:
        break

print(time() - start_time)
