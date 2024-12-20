import numpy as np
from ahrs.filters import EKF

def updateOrientation(q, a, g, m):
    ekf = EKF(frequency=100, noises=[0.1**2, 0.9**2, 0.7**2], q0=[1.0, 0.0, 0.0, 0.0])
    return ekf.update(q, g, a)