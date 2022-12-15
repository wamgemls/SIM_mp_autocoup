import scipy.interpolate as interpolate

import matplotlib.pyplot as plt
import numpy as np

#x = np.arange(11)
#y = np.sin(x)


    



x = np.array([1, 16.1])
y = np.array([0, -15.1])

xln = np.arange(-20,20, 0.001)

cs = interpolate.CubicSpline(x, y, bc_type=((1, 0), (1, -9)))
fig, ax = plt.subplots(figsize=(6, 6))

curv=cs(xln,2)/((1+((cs(xln,1)**2)))**(3/2))





ax.plot(x, y, 'o', label='data')
ax.plot(xln,cs(xln), label='test')
ax.plot(xln,curv,label='curvature')

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.grid(visible=True)
ax.legend(loc='lower left', ncol=2)


plt.show()




