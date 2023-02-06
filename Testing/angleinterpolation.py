import numpy as np






def angle_interp(self,x23, xp, yp):

    y23 = []

    for x3 in x23:

        x1 = 0
        x2 = 0
        x1_elem = None
        x2_elem = None

        i=0
        while i<len(xp):
            if xp[i] <= x3:
                x1 = xp[i]
                x1_elem = i
            i+=1
            
        j=len(xp)-1
        while j > 0:
            if xp[j] > x3:
                x2 = xp[j]
                x2_elem = j
            j-=1

        if x3 < x1:
            y23.append(yp[0])
        elif x3 > x2:
            y23.append(yp[-1])
        else:
            y23.append(self.calc_lin_interpol_angle(x1,x2,yp[x1_elem],yp[x2_elem],x3))

    return y23

def calc_lin_interpol_angle(x1,x2,y1,y2,x3):

    y1 = (np.rad2deg(y1)+360)%360
    y2 = (np.rad2deg(y2)+360)%360

    max_v = max(y1,y2)
    min_v = min(y1,y2)

    propA = max_v-min_v
    propB = 360-propA
    propF = min(propA,propB)

    xp = [x1,x2]
    fp = [0.0,propF]

    delta = np.interp(x3,xp,fp)
    
    #self.calc_lin_interpol(x1,x2,0,propF,x3)

    if y1 <= y2:

        if propF == propA:
            interpolated_v = min_v+delta
        elif propF == propB:
            interpolated_v = max_v+delta

    else:

        if propF == propA:
            interpolated_v = max_v-delta
        elif propF == propB:
            interpolated_v = min_v-delta
    
    interpolated_v = np.deg2rad(interpolated_v)

    return angle_interval(interpolated_v)
        
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

print(np.rad2deg(calc_lin_interpol_angle(0,1,np.deg2rad(20),np.deg2rad(350),0.5)))