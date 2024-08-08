import trimesh
import open3d as o3d
import pymeshlab

def ply2obj(file_path):
    '''
    Convert a ply file to an obj file
    args:
        file_path: str, path to the ply file
    return:
        export_path: str, path to the obj file
    '''

    mesh = trimesh.load(file_path)
    print("Original Vertices:")
    print(mesh.vertices)
    original_faces_count = len(mesh.faces)
    target_faces_count = int(original_faces_count * 0.5)
    simplified_mesh = mesh.simplify_quadric_decimation(target_faces_count)

    # Export the cleaned mesh to an OBJ file
    export_path = file_path.replace('.ply', '.obj')
    simplified_mesh.export(export_path)
    newmesh = trimesh.load(export_path)
    print(newmesh.vertices)
    return export_path


def pcd2ply(file_path):
    '''
    Convert a pcd file to a ply file
    args:
        file_path: str, path to the pcd file
    return:
        export_path: str, path to the ply file
    '''
    pcd = o3d.io.read_point_cloud(file_path)
    export_path = file_path.replace('.pcd', '.ply')

    o3d.io.write_point_cloud(export_path, pcd)
    return export_path

    

if __name__ == "__main__":
    # mesh_path = 'plydoc/1.pcd'
    # export_path = pcd2ply(mesh_path)
    # ply2obj(export_path)
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh('plydoc/1.ply')
    ms.create_noisy_isosurface(resolution=128)
    ms.save_current_mesh('plydoc/1.obj')







    

