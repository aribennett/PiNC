from time import time, time_ns, sleep

start_time = time()
interval_time = time_ns()
count = 0
while True:
    if time_ns() - interval_time > 1000000:
        count += 1
        interval_time = time_ns()
    if count == 1000:
        break

print(time() - start_time)
