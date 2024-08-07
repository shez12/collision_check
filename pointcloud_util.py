import trimesh

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


if __name__ == "__main__":
    mesh_path = 'plydoc/mesh9.ply'
    ply2obj(mesh_path)


    







    

