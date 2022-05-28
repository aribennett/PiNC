# from path_planning import get_corners, calculate_lerp
# from matplotlib import pyplot as plt
# import numpy as np
# box = np.array([[0, 0, 0, 0], [10, 0, 0, 5], [2, .5, 0, 4], [1, .5, 0, 9], [0, 1, 0, 4], [0, 0, 0, 8]])

# lerp, distance, speed, acceleration, time = calculate_lerp(box[0], box[1], 0, box[1][3], 0)
# plt.plot(time, distance)
# plt.plot(time, speed)
# plt.plot(time, acceleration / 30)
# plt.show()
from path_planning.gcode_solver import GcodeSolver

XY_MM_PER_RAD = 6.36619783227
Z_MM_PER_RAD = 0.63661977236
E_MM_PER_RAD = .85
FINE_Z = 29.7

# ------ Debug Variables --------
errorx = 0
errory = 0
# -------------------------------

jog_controller = None

with open('vase_helix.gcode', 'r') as f:
    gcode = f.read()

path_planner = GcodeSolver(gcode, start_position=[-XY_MM_PER_RAD, -XY_MM_PER_RAD, 0])
print(max(path_planner.time_array)/3600)
