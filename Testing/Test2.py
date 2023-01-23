import matplotlib.pyplot as plt
import numpy as np
import matplotlib

@staticmethod
def calc_lin_interpol(x1,x2,y1,y2,x3):
    m = (y2-y1)/(x2-x1)   
    return (m*x3)+(y1-(m*x1))

@staticmethod
def angle_interval(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi

def calc_lin_interpol_angle(x1,x2,y1,y2,x3):

    y1 = (np.rad2deg(y1)+360)%360
    y2 = (np.rad2deg(y2)+360)%360

    max_v = max(y1,y2)
    min_v = min(y1,y2)

    propA = max_v-min_v
    propB = 360-propA
    propF = min(propA,propB)
    delta = calc_lin_interpol(x1,x2,0,propF,x3)

    if propF == propA:
        interpolated_v = min_v+delta
    elif propF == propB:
        interpolated_v = max_v+delta
    
    interpolated_v = np.deg2rad(interpolated_v)

    return angle_interval(interpolated_v)

print(calc_lin_interpol_angle(2,4,-0.5,1,3))


