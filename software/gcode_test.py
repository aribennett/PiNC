import numpy as np
from matplotlib import pyplot as plt
from path_planning.gcode_solver import GcodeSolver
from path_planning import get_corners


# open gcode file and store contents as variable
with open('111cube.gcode', 'r') as f:
    gcode = f.read()

path_planner = GcodeSolver(gcode)

TIME = 30
current_time = np.linspace(0, TIME, num=TIME*10000)
xy = path_planner.output_array[:300, :2]
e = path_planner.output_array[:, 3]
plt.plot(e)
# plt.plot(xy[:, 0], xy[:, 1])
# position, velocity = path_planner.get_solution(current_time)
# for item in path_planner.time_array:
#     print(item)
# plt.plot(position[:, 0], position[:, 1])
# print(np.array(position[:, :2]))
# start, end, corners = get_corners(xy)
# for i, corner in enumerate(corners):
#     np_corner = corner.transpose()
#     plt.plot(np_corner[0], np_corner[1], c='r')

# plt.gca().set_aspect("equal")
plt.show()
