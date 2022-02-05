from path_planning import get_corners, calculate_lerp
from matplotlib import pyplot as plt
import numpy as np
box = np.array([[0, 0, 0, 0], [10, 0, 0, 5], [2, .5, 0, 4], [1, .5, 0, 9], [0, 1, 0, 4], [0, 0, 0, 8]])

lerp, distance, speed, acceleration, time = calculate_lerp(box[0], box[1], 0, box[1][3], 0)
plt.plot(time, distance)
plt.plot(time, speed)
plt.plot(time, acceleration / 30)
plt.show()
