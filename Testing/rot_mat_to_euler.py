from scipy.spatial.transform import Rotation as R
import numpy as np

r = R.from_matrix([[-0.014338,0.999382,0.032588],
			         [-0.573412,0.018413,-0.819035],
			         [-0.819057,-0.030462,0.572867]])

print(r.as_euler('xyz',degrees=True))
