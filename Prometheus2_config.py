import numpy as np

config = {
    "l1" : 1.4,
    "l2a" : 0.4,
    "l2b" : 0.7,
    "l3" : 0.4,
    "theta0" : 0.0,
    "theta1" : 0.0,
    "theta2a" : np.pi/2,
    "theta2b" : np.pi/2,
    "theta3" : 0.0,
    "theta0_bounds": [-np.pi/2, np.pi/2],
    "theta1_bounds": [0, np.pi],
    "theta2a_bounds": [-np.pi/2, np.pi/2],
    "theta3_bounds": [-np.pi/2, np.pi/2]
}
