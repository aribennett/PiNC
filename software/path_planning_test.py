from path_planning import get_corners
from matplotlib import pyplot as plt
import numpy as np
box = [[0, 0, 0, 0, 0], [1, 0, .2, 2000, 2], [2, .5, .3, 3000, 3], [1, .5, .6, 4000, 1], [0, 1, 1, 5000, 1], [0, 0, 0, 6000, .5]]

path, distance, time, speed, acc, tangent = get_corners(box)
raw = np.array(box).transpose()

ax = plt.axes()
output = path.transpose()
# plt.plot(distance, output[0])
# plt.plot(distance, output[1])
# plt.plot(distance, output[2])
# for t, p in zip(tangent, path[:, :3]):
#     tan = np.array([p, t/10+p]).T
#     ax.plot(tan[0], tan[1])
ax.plot(time, distance)
ax.plot(time, speed)
ax.plot(time, np.clip(acc/10, -2, 2))
# ax.scatter(output[0], output[1], s=1)
plt.show()


# for corner in corners:
# print(start, end, corners)
