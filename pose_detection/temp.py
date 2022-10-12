import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
TC2B = np.eye(4)
R = Rotation.from_euler("XYZ",[45,45,0], degrees=True).as_matrix()
TC2B[:3,:3] = R
TC2B[:3,3] = np.array([1,2,3])
print (TC2B)

TB2A = np.eye(4)
R = Rotation.from_euler("XYZ",[75,0,-45], degrees=True).as_matrix()
TB2A[:3,:3] = R
TB2A[:3,3] = np.array([1,5,3])

print(TB2A)
origin_c_in_a = TB2A@TC2B
print(origin_c_in_a)


point_P_c_in_a = TB2A @ TC2B @ np.array([2,0,0,1]).reshape(4,1)
print(point_P_c_in_a)

