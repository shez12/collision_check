import open3d as o3d
import numpy as np
import pymeshfix


class pointcloud:
    def __init__(self,pcd) -> None:
        self.pcd = pcd


    def clean_mesh(self,mesh):

        v,f = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        vclean, fclean = pymeshfix.clean_from_arrays(v, f)
        # create a new mesh
        new_mesh = o3d.geometry.TriangleMesh()
        new_mesh.vertices = o3d.utility.Vector3dVector(vclean)
        new_mesh.triangles = o3d.utility.Vector3iVector(fclean)
        
        return new_mesh


    def create_mesh(self):
        # Estimate normals for the point cloud
        self.pcd.estimate_normals()

        # Compute nearest neighbor distances to estimate an average distance
        distances = self.pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)

        # Calculate a radius for the rolling ball algorithm
        radius = 1.5 * avg_dist

        # Create a triangular mesh using the Ball Pivoting algorithm from Open3D
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            self.pcd,
            o3d.utility.DoubleVector([radius, radius * 2]))

        return self.clean_mesh(mesh)
    
    def get_pos(self):
        # get mesh position
        return self.pcd.get_center()


    

    def visualize(self, mesh):
        # Set color of the mesh to red
        mesh.paint_uniform_color([1.0, 0.0, 0.0])
        # Visualization using Open3D
        o3d.visualization.draw_geometries([mesh])


    def export(self, mesh, filename):
       # save as .obj file
        o3d.io.write_triangle_mesh(filename, mesh)
        







    

