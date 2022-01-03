import numpy as np

# DISTANCE AT WHICH TO START CORNER INTERPOLATION
CORNER_DISTANCE = .1
CORNER_RESOLUTION = 100
SEGMENT_RESOLUTION = 1000
MIN_SEGMENT_RESOLUTION = .001
BEZIER_POINTS = np.linspace(0, 1, CORNER_RESOLUTION+1).reshape((-1, 1))[1:]


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


def calculate_lerp(s0, s1, input_speed, output_speed, cruise_speed):
    distance = np.linalg.norm((s0 - s1)[:3])
    segment_count = min(int(distance/MIN_SEGMENT_RESOLUTION), SEGMENT_RESOLUTION)
    t = np.linspace(0, 1, segment_count+1).reshape((-1, 1))[1:]
    lerp = s0 + (s1 - s0)*t
    lerp[:, 4] = np.sqrt(lerp[:, 4])
    return lerp


def get_corners(segment_list):
    start = segment_list[0]
    end = segment_list[-1]
    corner_list = []
    corner_list = np.zeros((len(segment_list) * (CORNER_RESOLUTION+SEGMENT_RESOLUTION) + 2, 5))
    corner_list[0] = start
    corner_index = 1
    iterator = zip(
        np.array(segment_list[:-2]),
        np.array(segment_list[1:-1]),
        np.array(segment_list[2:]),
        np.array(segment_list)[1:-1, 4],
        np.array(segment_list)[2:, 4]
    )

    # exit_feed = segment_list[1][4]
    exit_feed = 0
    for s0, s1, s2, f0, f2 in iterator:
        f1 = (f0 + f2)/2
        d0 = s0 - s1
        d2 = s2 - s1
        d0_n = np.linalg.norm(d0[:3])
        d2_n = np.linalg.norm(d2[:3])
        if d2_n != 0 and d0_n != 0:
            s0_u = d0/d0_n
            s2_u = d2/d2_n
            nominal_corner = CORNER_DISTANCE
            min_corner = min(d0_n/2, d2_n/2)
            if min_corner < CORNER_DISTANCE:
                nominal_corner = min_corner
            dot_product = np.dot(s0_u[:3], s2_u[:3])
            angle = np.arccos(dot_product)
            current_deviation = np.sin(angle)*nominal_corner
            s0_p = s0_u * current_deviation + s1
            s2_p = s2_u * current_deviation + s1
            s0_p[4] = f1
            s1[4] = f1
            s2_p[4] = f1
            lerp_start = np.copy(corner_list[corner_index-1])
            lerp_start[4] = exit_feed**2
            lerp_end = np.copy(s0_p)
            lerp_end[4] = f1**2
            lerp = calculate_lerp(lerp_start, lerp_end)
            corner = calculate_bezier(s0_p, s1, s2_p)
            np.copyto(corner_list[corner_index:corner_index + lerp.shape[0]], lerp)
            corner_index += lerp.shape[0]
            np.copyto(corner_list[corner_index:corner_index + CORNER_RESOLUTION], corner)
            corner_index += CORNER_RESOLUTION
            exit_feed = f1
        else:
            corner_list[corner_index] = s1
            corner_index += 1

    lerp_start = np.copy(corner_list[corner_index-1])
    lerp_start[4] = exit_feed**2
    lerp = calculate_lerp(lerp_start, end)
    np.copyto(corner_list[corner_index:corner_index+lerp.shape[0]], lerp)
    corner_index += lerp.shape[0]
    path = np.copy(corner_list[:corner_index])
    difference = np.diff(path.transpose()).transpose()[:, :3]
    dist = np.linalg.norm(difference, axis=1)
    norm = dist.reshape((-1, 1))
    tangent = difference/norm
    speed = path[1:, 4]

    time = norm/speed.reshape((-1, 1))
    distance = np.zeros(path.shape[0])
    time_arr = np.zeros(path.shape[0])
    tangent_arr = np.zeros((path.shape[0], 3))
    speed_arr = np.zeros(path.shape[0])

    np.copyto(tangent_arr[:-1], tangent)
    np.copyto(time_arr[1:], np.cumsum(time))
    np.copyto(distance[1:], np.cumsum(dist))
    np.copyto(speed_arr[1:], speed)

    acc = np.diff(speed_arr)/time.T
    acc_arr = np.zeros(path.shape[0])
    np.copyto(acc_arr[1:], acc)

    return path, distance, time_arr, speed_arr, acc_arr, tangent_arr
