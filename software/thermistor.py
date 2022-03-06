import numpy as np
resistance = 2.7
thermistor_curve = np.array([[-50, 8887],
                            [-30, 2156],
                            [-10, 623.2],
                            [0, 354.6],
                            [10, 208.8],
                            [25, 100.00],
                            [40, 50.90],
                            [50, 33.45],
                            [60, 22.48],
                            [80, 10.8],
                            [85, 9.094],
                            [100, 5.569],
                            [120, 3.058],
                            [140, 1.770],
                            [160, 1.074],
                            [180, 0.6763],
                            [200, 0.4452],
                            [220, 0.3016],
                            [240, 0.2104],
                            [260, 0.1507],
                            [280, 0.1105],
                            [300, 0.08278]]).T


def get_thermistor_temp(adc):
    v_therm = 3.3*(adc/1024)
    v = 3.3 - v_therm
    r_therm = v_therm/(v/resistance)
    temp = np.interp(-r_therm, -thermistor_curve[1], thermistor_curve[0])
    return temp, r_therm
