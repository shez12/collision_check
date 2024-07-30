import trimesh
import pyrender
import numpy as np
from pointcloud_util import pointcloud
import pyvista as pv
import pymeshfix
import open3d as o3d
from scipy.spatial.transform import Rotation

# Load the mesh from the .obj file
infile = "plydoc/mesh4.obj"
mesh = trimesh.load(infile)
v,f = mesh.vertices, mesh.faces

meshfix = pymeshfix.MeshFix(v, f)

# meshfix.repair()

meshfix.plot()



meshfix.save("plydoc/mesh4_fixed.obj")


