import numpy as np
from gcodeparser import GcodeParser
from . import get_corners


class GcodeSolver(object):
    def __init__(self, gcode_str, start_position=[0, 0, 0]):
        lines = GcodeParser(gcode_str).lines    # get parsed gcode lines
        x = start_position[0]
        y = start_position[0]
        z = start_position[0]
        e = 0
        f = 0
        t = 0
        vx = 0
        vy = 0
        vz = 0
        ve = 0

        self.motion_list = [[x, y, z, e, f, t, vx, vy, vz, ve]]

        for line in lines:
            if line.command == ('G', 1):
                prev = self.motion_list[-1]
                if 'X' in line.params:
                    x = line.params['X']
                if 'Y' in line.params:
                    y = -line.params['Y']
                if 'Z' in line.params:
                    z = line.params['Z']
                if 'E' in line.params:
                    e = -line.params['E']
                if 'F' in line.params:
                    f = line.params['F']/60  #  Convert to

                output = True

                if x != prev[0] or y != prev[1] or z != prev[2]:
                    dx = x - prev[0]
                    dy = y - prev[1]
                    dz = z - prev[2]
                    distance = np.sqrt(np.square(dx) + np.square(dy) + np.square(dz))
                    dt = distance/f
                    vx = dx/dt
                    vy = dy/dt
                    vz = dz/dt
                    t += dt

                elif e != self.motion_list[-1][3]:
                    # solely extruder move
                    vx = 0
                    vy = 0
                    vz = 0
                    t += abs(prev[3]-e)/f

                else:
                    # Feed rate change
                    output = False

                if output:
                    self.motion_list.append([x, y, z, e, f, t, vx, vy, vz, ve])

        self.output_array = np.array(self.motion_list)
        self.time_array = self.output_array[:, 5]
        self.posistion_array = self.output_array[:, 0:4]
        self.velocity_array = self.output_array[:, 6:10]


    def get_solution(self, time):
        insert_index = np.searchsorted(self.time_array, time)
        interp = (time - self.time_array[insert_index-1])/(self.time_array[insert_index] - self.time_array[insert_index-1])
        axis_interp = np.transpose(np.tile(interp, (4, 1)))
        position = axis_interp*(self.posistion_array[insert_index] - self.posistion_array[insert_index-1]) + self.posistion_array[insert_index-1]
        velocity = self.velocity_array[insert_index]
        return position, velocity
