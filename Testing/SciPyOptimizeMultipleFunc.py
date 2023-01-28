import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from scipy.integrate import quad
from scipy.optimize import fsolve

tg = 0
sg = 15

vx_pos = 0.41
ego_vx_pos = 0

cc_time = 0.5

acc_profile = lambda x: (((vx_pos-ego_vx_pos)/cc_time)*x)+ego_vx_pos
const_profile = lambda x: (vx_pos*x)
dec_profile = lambda x: ((-vx_pos/cc_time)*(x))+vx_pos

ds1,_ = quad(acc_profile,0,cc_time)
ds3,_ = quad(dec_profile,0,cc_time)
ds2 = sg-ds1-ds3

dt1 = cc_time
dt2 = ds2/vx_pos
dt3 = cc_time

len_on_traj = 0.1

def integrand_acc(t):
    return (((vx_pos-ego_vx_pos)/cc_time)*t)+ego_vx_pos

def integrand_const(t):
    return (vx_pos)

def integrand_dec(t):
    return ((-vx_pos/cc_time)*(t))+vx_pos

def func_acc(t):
    integral,err = quad(integrand_acc,0,t)
    return len_on_traj-integral

def func_const(t):
    integral,err = quad(integrand_const,0,t)
    return len_on_traj-integral

def func_dec(t):
    integral,err = quad(integrand_dec,0,t)
    return len_on_traj-integral

vfunc_acc = np.vectorize(func_acc)
vfunc_const = np.vectorize(func_const)
vfunc_dec = np.vectorize(func_dec)

res, = fsolve(vfunc_dec,0.01)
print(res)