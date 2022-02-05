import numpy as np

# DISTANCE AT WHICH TO START CORNER INTERPOLATION
CORNER_DISTANCE = .1
CORNER_RESOLUTION = 10
MAX_ACCELERATION = 30  # mm/s^2
MIN_CORNER_SPEED = 1  # mm/s
LERP_TIME_INTERVAL = 0.005  # s

BEZIER_POINTS = np.linspace(0, 1, CORNER_RESOLUTION, endpoint=False).reshape((-1, 1))


def calculate_bezier(s0, s1, s2):
    """Calculate a quadratic bezier through one corner

    Args:
        s0 (array): first point
        s1 (array): second point
        s2 (array): third point
        t (array): interpolation array

    Returns:
        array: list of interpolated positions
    """
    corner = s1 + (s0 - s1) * np.square(1 - BEZIER_POINTS) + (s2 - s1) * np.square(BEZIER_POINTS)
    return corner


def calculate_lerp(s0, s1, input_speed=0, cruise_speed=1, output_speed=0):
    #  d = .5at^2 + vt
    #  0 = .5at^2 + vt - d
    #  v = at

    total_distance = np.linalg.norm((s0 - s1)[:3])

    accel_time = (cruise_speed - input_speed)/MAX_ACCELERATION
    accel_dist = .5 * MAX_ACCELERATION * np.square(accel_time) + input_speed*accel_time

    decel_time = (cruise_speed - output_speed)/MAX_ACCELERATION
    decel_dist = .5 * MAX_ACCELERATION * np.square(decel_time) + output_speed*decel_time

    cruise_dist = total_distance - accel_dist - decel_dist
    cruise_time = cruise_dist / cruise_speed

    total_time = accel_time + decel_time + cruise_time
    points = int(total_time/LERP_TIME_INTERVAL)
    time_array = np.linspace(0, total_time, points)

    accel_mask = time_array < accel_time
    accel_speed = (time_array * MAX_ACCELERATION + input_speed) * accel_mask
    accel_distance = (.5 * MAX_ACCELERATION * np.square(time_array) + input_speed * time_array) * accel_mask
    accel_array = np.ones(points) * MAX_ACCELERATION * accel_mask

    cruise_mask = time_array > accel_time
    cruise_mask *= time_array <= accel_time + cruise_time
    cruise_speed_arr = np.ones(points) * cruise_speed * cruise_mask
    cruise_distance = ((time_array - accel_time) * cruise_speed + accel_dist) * cruise_mask

    decel_mask = time_array > accel_time + cruise_time
    decel_time = total_time - time_array - decel_time
    decel_speed = (decel_time * MAX_ACCELERATION + cruise_speed) * decel_mask
    decel_distance = (.5 * -MAX_ACCELERATION * np.square(decel_time) + total_distance - decel_dist - cruise_speed * decel_time) * decel_mask
    decel_array = np.ones(points) * -MAX_ACCELERATION * decel_mask

    acceleration = accel_array + decel_array
    speed = accel_speed + cruise_speed_arr + decel_speed
    distance = accel_distance + cruise_distance + decel_distance
    lerp = (distance/total_distance).reshape(-1, 1)
    lerp = (s0 + (s1 - s0)*lerp).T
    lerp[3, :] = speed

    return lerp, distance, speed, acceleration, time_array


def get_corners(segment_list):
    start = segment_list[0]
    end = segment_list[-1]
    corner_list = []
    corner_list = np.zeros((len(segment_list) * (CORNER_RESOLUTION+SEGMENT_RESOLUTION) + 2, 4))
    corner_index = 0
    iterator = zip(
        np.array(segment_list[:-2]),
        np.array(segment_list[1:-1]),
        np.array(segment_list[2:])
    )

    exit_point = start
    entry_speed = 0
    for s0, s1, s2 in iterator:
        f0 = s1[3]
        f2 = s2[3]
        d0 = s0 - s1
        d2 = s2 - s1
        d0_n = np.linalg.norm(d0[:3])
        d2_n = np.linalg.norm(d2[:3])
        s0_u = d0/d0_n
        s2_u = d2/d2_n
        nominal_corner = CORNER_DISTANCE
        min_corner = min(d0_n/2, d2_n/2)
        if min_corner < CORNER_DISTANCE:
            nominal_corner = min_corner
        dot_product = np.dot(s0_u[:3], s2_u[:3])

        angle = np.arccos(dot_product)
        corner_speed_scale = angle/np.pi
        f1 = max(min(f0, f2) * corner_speed_scale, MIN_CORNER_SPEED)

        current_deviation = np.sin(angle)*nominal_corner
        s0_p = s0_u * current_deviation + s1
        s2_p = s2_u * current_deviation + s1
        s0_p[3] = f1
        s1[3] = f1
        s2_p[3] = f1
        lerp = calculate_lerp(exit_point, s0_p, entry_speed, f1, f0)
        corner = calculate_bezier(s0_p, s1, s2_p)
        np.copyto(corner_list[corner_index:corner_index + lerp.shape[0]], lerp)
        corner_index += lerp.shape[0]
        np.copyto(corner_list[corner_index:corner_index + CORNER_RESOLUTION], corner)
        corner_index += CORNER_RESOLUTION
        exit_point = s2_p
        entry_speed = f1

    # lerp_start = np.copy(corner_list[corner_index-1])
    # lerp_start[4] = exit_feed**2
    # lerp = calculate_lerp(lerp_start, end)
    # np.copyto(corner_list[corner_index:corner_index+lerp.shape[0]], lerp)
    # corner_index += lerp.shape[0]
    path = np.copy(corner_list[:corner_index])
    difference = np.diff(path.transpose()).transpose()[:, :3]
    dist = np.linalg.norm(difference, axis=1)
    norm = dist.reshape((-1, 1))
    tangent = difference/norm
    # speed = path[1:, 4]

    # time = norm/speed.reshape((-1, 1))
    # distance = np.zeros(path.shape[0])
    # time_arr = np.zeros(path.shape[0])
    # tangent_arr = np.zeros((path.shape[0], 3))
    # speed_arr = np.zeros(path.shape[0])

    # np.copyto(tangent_arr[:-1], tangent)
    # np.copyto(time_arr[1:], np.cumsum(time))
    # np.copyto(distance[1:], np.cumsum(dist))
    # np.copyto(speed_arr[1:], speed)

    # acc = np.diff(speed_arr)/time.T
    # acc_arr = np.zeros(path.shape[0])
    # np.copyto(acc_arr[1:], acc)
    return path, tangent
    return path, distance, time_arr, speed_arr, acc_arr, tangent_arr
