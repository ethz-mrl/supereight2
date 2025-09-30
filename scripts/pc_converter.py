import argparse
import trimesh
import open3d as o3d

parser = argparse.ArgumentParser(description="Process and visualize a point cloud.")
parser.add_argument("input_file", help="Path to the input .ply file")
args = parser.parse_args()

try:
    mesh_sample = o3d.io.read_point_cloud(args.input_file)
    mesh_down = mesh_sample.voxel_down_sample(0.05)
    mesh_down.estimate_normals()

    o3d.visualization.draw_geometries([mesh_down],
                                    zoom=0.4559,
                                    front=[0.6452, -0.3036, -0.7011],
                                    lookat=[1.9892, 2.0208, 1.8945],
                                    up=[-0.2779, -0.9482, 0.1556])

    output_name = args.input_file[:-4] + ".pcd"
    o3d.io.write_point_cloud(output_name, mesh_down)

except Exception as e:
    print(f"Error loading point cloud: {e}")