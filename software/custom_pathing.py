from path_planning.gcode_solver import GcodeSolver
from matplotlib import pyplot as plt
import numpy as np
with open('gcode_examples/box-test.gcode', 'r') as f:
    gcode = f.read()

path_planner = GcodeSolver(gcode, start_position=[0, 0, 0])
x = path_planner.posistion_array[:, 0]
y = path_planner.posistion_array[:, 1]
z = path_planner.posistion_array[:, 2]
e = path_planner.posistion_array[:, 3]

ve = path_planner.velocity_array[:, 3]
xyz_normal = path_planner.posistion_array[1:] - path_planner.posistion_array[:-1]

# print(np.linalg.norm(xyz_normal, axis=1))
xyz_normal = xyz_normal / np.linalg.norm(xyz_normal, axis=1)[:, np. newaxis]

print(xyz_normal)
plt.plot(ve)
# plt.plot(x, y)
plt.show()
