import pandas as pd
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Slerp, Rotation as R
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


# ================= CONFIGURATION =================
MESH_FILE = '/home/wiss/weji/Downloads/scoutdi_test/mesh_396.ply'
FILE_TRAJ = '/usr/wiss/weji/Datasets/scoutdi_dataset/trajectory.csv'      # map -> base_link
FILE_UT_POSE = '/usr/wiss/weji/Datasets/scoutdi_dataset/ut_pose_dynamic.csv' # base_link -> ut_face
FILE_UT_MEAS = '/usr/wiss/weji/Datasets/scoutdi_dataset/ut_measurement.csv'  # timestamp, thickness
OUTPUT_FILE = '/usr/wiss/weji/Datasets/scoutdi_dataset/ut_global_registered.csv'
OUTPUT_PCD = '/usr/wiss/weji/Datasets/scoutdi_dataset/ut_measurements_colored.ply'

COLOR_MAP = 'Reds'  # 'jet', 'viridis', 'hot', etc.

# ================= HELPER CLASSES =================

class PoseInterpolator:
    def __init__(self, df):
        """
        Expects a DataFrame with columns: ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
        """
        self.times = df['timestamp'].values
        self.pos = df[['tx', 'ty', 'tz']].values
        self.quat = df[['qx', 'qy', 'qz', 'qw']].values
        
        # 1. Position Interpolation (Linear)
        self.interp_pos = interp1d(self.times, self.pos, axis=0, kind='linear', fill_value="extrapolate")
        
        # 2. Rotation Interpolation (SLERP)
        self.rotations = R.from_quat(self.quat)
        self.slerp = Slerp(self.times, self.rotations)

    def get_pose(self, query_time):
        # Clip time to range to avoid Slerp errors (extrapolation is risky for rotation)
        t = np.clip(query_time, self.times[0], self.times[-1])
        
        # Interpolate
        p = self.interp_pos(t)
        r = self.slerp([t]).as_matrix()[0] # 3x3 rotation matrix
        
        # Construct 4x4 Matrix
        T = np.eye(4)
        T[:3, :3] = r
        T[:3, 3] = p
        return T

# ================= MAIN EXECUTION =================

def main():
    print("Loading data...")
    try:
        df_traj = pd.read_csv(FILE_TRAJ)
        df_ut_pose = pd.read_csv(FILE_UT_POSE)
        df_meas = pd.read_csv(FILE_UT_MEAS)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return

    # Initialize Interpolators
    print("Building interpolators...")
    # Clean column names (strip whitespace just in case)
    df_traj.columns = [c.strip() for c in df_traj.columns]
    df_ut_pose.columns = [c.strip() for c in df_ut_pose.columns]
    
    # Sort by timestamp to ensure interpolation works
    df_traj = df_traj.sort_values('timestamp')
    df_ut_pose = df_ut_pose.sort_values('timestamp')
    
    traj_interp = PoseInterpolator(df_traj)
    ut_pose_interp = PoseInterpolator(df_ut_pose)

    print("Registering measurements to Global Map Frame...")
    
    global_points = []
    thickness_vals = []
    valid_rows = []

    for idx, row in df_meas.iterrows():
        ts = row['timestamp']
        thickness = row['thickness_m']
        
        # 1. Get Robot Pose at this time (Map -> Base)
        # Check if timestamp is within range (with 0.5s buffer)
        if ts < df_traj['timestamp'].min() or ts > df_traj['timestamp'].max():
            continue
            
        T_map_base = traj_interp.get_pose(ts)
        
        # 2. Get Probe Pose at this time (Base -> Face)
        T_base_face = ut_pose_interp.get_pose(ts)
        
        # 3. Compute Global Pose (Map -> Face)
        # T_map_face = T_map_base * T_base_face
        T_map_face = np.dot(T_map_base, T_base_face)
        
        # Extract Position (x, y, z)
        global_pos = T_map_face[:3, 3]
        
        global_points.append(global_pos)
        thickness_vals.append(thickness)
        
        # Save for CSV
        valid_rows.append([ts, thickness, global_pos[0], global_pos[1], global_pos[2]])

    # --- SAVE RESULTS ---
    print(f"Registered {len(valid_rows)} / {len(df_meas)} measurements.")
    df_result = pd.DataFrame(valid_rows, columns=['timestamp', 'thickness', 'x', 'y', 'z'])
    df_result.to_csv(OUTPUT_FILE, index=False)
    print(f"Saved to {OUTPUT_FILE}")

    # --- 3D OUTPUT GENERATION ---
    if not global_points:
        print("No valid points found. Exiting.")
        return

    # Create Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(global_points))

    # Colorize
    vals = np.array(thickness_vals)
    norm = (vals - vals.min()) / (vals.max() - vals.min() + 1e-9)
    cmap = plt.get_cmap(COLOR_MAP)
    colors = cmap(norm)[:, :3] # RGB
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Save PLY file
    print("Saving 3D Point Cloud file...")
    o3d.io.write_point_cloud(OUTPUT_PCD, pcd)
    print(f"3D File saved: {OUTPUT_PCD}")


if __name__ == "__main__":
    main()