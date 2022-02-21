z0 = (30, 4)
z1 = (8, 45)
z2 = (58, 45)
center_home = (25, 25)


def corexy_transform(x_nominal, y_nominal):
    return -y_nominal - x_nominal, y_nominal - x_nominal


def corexy_inverse(m0, m1):
    return (-m0 - m1)/2, (m1 - m0)/2
