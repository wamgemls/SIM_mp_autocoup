import matplotlib.pyplot as plt 
import numpy as np
import pyclothoids as pyclothoid

import math

poseA = [0.5,0.5,np.deg2rad(-5),0.0]
poseB = [3,3,np.deg2rad(-15),0.0]

length = 2
width = 0.2

g2_clothoid = pyclothoid.SolveG2(poseA[0],poseA[1],poseA[2],poseA[3],poseB[0],poseB[1],poseB[2],poseB[3])

plt.rc('text', usetex=True)
plt.rc('font', family='serif')


figure, axis = plt.subplots()
axis.grid()

axis.arrow(poseA[0] - (length/2 * math.cos(poseA[2])), poseA[1]-(length/2 * math.sin(poseA[2])), length * math.cos(poseA[2]), length * math.sin(poseA[2]),fc="k", ec="k", head_width=width, head_length=width,linewidth=0.6)
axis.arrow(poseB[0] - (length/2 * math.cos(poseB[2])), poseB[1]-(length/2 * math.sin(poseB[2])), length * math.cos(poseB[2]), length * math.sin(poseB[2]),fc="k", ec="k", head_width=width, head_length=width,linewidth=0.6)

axis.plot(*g2_clothoid[0].SampleXY(500),linewidth=1.4)
axis.plot(*g2_clothoid[1].SampleXY(500),linewidth=1.4)
axis.plot(*g2_clothoid[2].SampleXY(500),linewidth=1.4)

axis.plot(poseA[0],poseA[1],color="k",marker="x")
axis.plot(poseB[0],poseB[1],color="k",marker="x")

plt.xlabel("x-axis",fontsize=13)
plt.ylabel("y-axis",fontsize=13)

axis.text(poseA[0],poseA[1]+0.25,r'$(x_0,y_0,\theta_0,\kappa_0)$',horizontalalignment='center',fontsize=13)
axis.text(poseB[0],poseB[1]+0.25,r'$(x_1,y_1,\theta_1,\kappa_1)$',horizontalalignment='center',fontsize=13)

axis.set_aspect('equal','datalim')

plt.savefig('g2interpolationproblem.png', dpi=300)
plt.show()