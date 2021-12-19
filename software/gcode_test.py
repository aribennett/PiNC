from gcodeparser import GcodeParser
import numpy as np
from matplotlib import pyplot as plt
from time import sleep, time
from gcode_solver import GcodeSolver
# open gcode file and store contents as variable
with open('box_gcode.gcode', 'r') as f:
    gcode = f.read()

path_planner = GcodeSolver(gcode)

TIME = 100
current_time = np.linspace(0, TIME, num=TIME*10000)
position, velocity = path_planner.get_solution(current_time)
for item in path_planner.time_array:
    print(item)
plt.plot(position[:, 0], position[:, 1])
plt.gca().set_aspect("equal")
plt.show()