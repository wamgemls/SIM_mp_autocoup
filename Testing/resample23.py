import numpy as np

path_res = 0.1
path23_res = 0.05
history_point_limit = 3

s_list = []
p1_list_x = []
p1_list_y = []

s_list_y = []

for i in np.arange(-path23_res*history_point_limit,(23-history_point_limit)*path23_res,path23_res):
    s_list.append(i+0.2)

for i in np.arange(0,100*path_res,path_res):
    p1_list_x.append(i)
    p1_list_y.append(np.sin(i))

s_list_y = np.interp(s_list,p1_list_x,p1_list_y)


y = np.mod(np.interp(x, xp, np.unwrap(fp, period=period)), period)




print("s")
