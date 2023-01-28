import matplotlib.pyplot as plt
import numpy as np
import matplotlib

@staticmethod
def trapez_integral(fa,fb,a,b):
    return ((b-a)/2) * (fa+fb)

@staticmethod
def divide_ZE(x,y):
    try:
        return x/y
    except ZeroDivisionError:
        return 0

@staticmethod
def angle_interval(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi

print(divide_ZE(-0,-0))


