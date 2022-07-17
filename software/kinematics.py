z0 = (-32, -1)
z1 = (-54, -45)
z2 = (-1, -45)
center_home = (-25, -25)


def corexy_transform(x_nominal, y_nominal):
    return y_nominal - x_nominal, - y_nominal - x_nominal


def corexy_inverse(m0, m1):
    return (-m1 - m0)/2, (m0 - m1)/2


if __name__ == '__main__':
    # Test all the quadrants
    print(corexy_transform(*corexy_inverse(20, 50)))
    print(corexy_transform(*corexy_inverse(-20, 50)))
    print(corexy_transform(*corexy_inverse(20, -50)))
    print(corexy_transform(*corexy_inverse(-20, -50)))
        
        