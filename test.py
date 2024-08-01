import trimesh
import pyrender
import numpy as np
from pointcloud_util import pointcloud
import pyvista as pv
import pymeshfix
import open3d as o3d
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

R = np.array([[0.2972,-0.6661,-0.6855], [-0.9546,-0.2223,-0.1984], [-0.02,0.714,-0.7005]])
print(R)
quat = matrix2quat(R)
print(quat)
# print(rpy2quat([0,0,-62]))