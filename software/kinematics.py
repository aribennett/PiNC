def corexy_transform(x_nominal, y_nominal):
    return -y_nominal - x_nominal ,y_nominal - x_nominal

def corexy_inverse(m0, m1):
    return  (-m0 - m1)/2, (m1 - m0)/2