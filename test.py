
import numpy as np
from scipy.spatial.transform import Rotation

#rpy to quaternion

def rpy2matrix(rpy):

    r = Rotation.from_euler('zyx', rpy, degrees=False)
    return r.as_matrix()

def matrix2quat(matrix):
    r = Rotation.from_matrix(matrix)
    return r.as_quat()

def rpy2quat(rpy):
    r = Rotation.from_euler('zyx', rpy, degrees=False)
    return r.as_quat()

R = np.array([[0.289 ,0.5945  ,  0.7503 ], [0.9566 ,  -0.148    ,-0.2512], [-0.03834,   0.7904  , -0.6115]])
print(R)
quat = matrix2quat(R)
print(quat)
# print(rpy2quat([0,0,-62]))