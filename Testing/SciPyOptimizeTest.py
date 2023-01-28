import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from scipy import integrate
from scipy.optimize import fsolve

vx = 0.41
ego_vx = 0

tg = 0
sg = 15


cc_time = 0.5

acc_profile = lambda x: (((vx-ego_vx)/cc_time)*x)+ego_vx
const_profile = lambda x: (vx*x)
dec_profile = lambda x: ((-vx/cc_time)*(x))+vx

len_on_traj = 0

def integrand_acc(t):
    return (((vx-ego_vx)/cc_time)*t)+ego_vx

def integrand_const(t):
    return (vx*t)


def func(t,func):
    integral,err = integrate.quad(func,0,t)
    return len_on_traj-integral
    
vfunc = np.vectorize(func)

#for x in np.arange(0,20,0.1):
    #path = x
T_solved = fsolve(vfunc,0.01)
print(T_solved)








