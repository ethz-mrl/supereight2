import argparse
import numpy as np
import trimesh
import open3d as o3d

parser = argparse.ArgumentParser(description="Process and visualize a point cloud.")
parser.add_argument("input_file", help="Path to the input .ply file")
args = parser.parse_args()

try:
    # -----------------------------
    # Vertex-colored point cloud
    # -----------------------------
    pc = o3d.io.read_point_cloud(args.input_file)
    pc_down = pc.voxel_down_sample(0.05)
    pc_down.estimate_normals()

    o3d.visualization.draw_geometries([pc_down])

    o3d.io.write_point_cloud(
        args.input_file[:-4] + "_vertex_colors.pcd",
        pc_down
    )

    # -----------------------------
    # TRUE face-colored point cloud
    # -----------------------------
    mesh = trimesh.load(args.input_file, process=False)

    if mesh.faces is None or mesh.visual.face_colors is None:
        raise ValueError("Mesh does not contain face colors.")

    # Sample points from surface + face indices
    n_face_samples = len(mesh.faces)
    face_centers, face_idx = trimesh.sample.sample_surface(
        mesh, n_face_samples
    )

    # Face colors: RGBA → RGB → float
    # Fix this: if there's vertex color, this one saves the vertex color.
    face_colors = mesh.visual.face_colors[face_idx][:, :3] / 255.0
    print(face_idx)
    print(mesh.visual.face_colors)

    pc_face = o3d.geometry.PointCloud()
    pc_face.points = o3d.utility.Vector3dVector(face_centers)
    pc_face.colors = o3d.utility.Vector3dVector(face_colors)
    pc_face.estimate_normals()

    o3d.visualization.draw_geometries([pc_face])

    o3d.io.write_point_cloud(
        args.input_file[:-4] + "_face_colors.pcd",
        pc_face
    )

    print("Saved:")
    print(" ", args.input_file[:-4] + "_vertex_colors.pcd")
    print(" ", args.input_file[:-4] + "_face_colors.pcd")

except Exception as e:
    print(f"Error: {e}")
