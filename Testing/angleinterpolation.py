import numpy as np

#test np unwrap

yaw_23 = np.unwrap(250,360)

def angle_interp(self,x23, xp, yp):

    y23 = []

    for x3 in x23:

        x1 = 0
        x2 = 0  

        i=-1
        while xp[i] <= x3:
            i+=1
            x1 = xp[i]
            
        j=len(yp)
        while xp[j] >= x3:
            j-=1
            x2 = xp[j]

        y23.append(x1,x2,yp[i],yp[i],x3)

        

    
def calc_lin_interpol_angle(self,x1,x2,y1,y2,x3):

    y1 = (np.rad2deg(y1)+360)%360
    y2 = (np.rad2deg(y2)+360)%360

    max_v = max(y1,y2)
    min_v = min(y1,y2)

    propA = max_v-min_v
    propB = 360-propA
    propF = min(propA,propB)
    delta = self.calc_lin_interpol(x1,x2,0,propF,x3)

    if propF == propA:
        interpolated_v = min_v+delta
    elif propF == propB:
        interpolated_v = max_v+delta
    
    interpolated_v = np.deg2rad(interpolated_v)

    return self.angle_interval(interpolated_v)
        
@staticmethod
def calc_lin_interpol(x1,x2,y1,y2,x3):
    m = (y2-y1)/(x2-x1)   
    return (m*x3)+(y1-(m*x1))

@staticmethod
def find_intersection(LineA_x1,LineA_x2,LineA_y1,LineA_y2,LineB_x1,LineB_x2,LineB_y1,LineB_y2):

    xdiff = (LineA_x1 - LineA_x2, LineB_x1 - LineB_x2)
    ydiff = (LineA_y1 - LineA_y2, LineB_y1 - LineB_y2)

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det((LineA_x1,LineA_y1),(LineA_x2,LineA_y2)),\
        det((LineB_x1, LineB_y1),(LineB_x2,LineB_y2)))

    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    return x,y

@staticmethod
def angle_interval(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi

print(yaw_23)