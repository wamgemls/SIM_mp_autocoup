from scipy.spatial.transform import Rotation as R
import numpy as np

m = [	[-0.999870,-0.015680,0.010166,],
		[0.015637,-0.999816,0.010879,],
		[-0.003964,0.011037,0.999889]]

r = R.from_matrix(m)


print(r.as_euler('xyz',degrees=False))
