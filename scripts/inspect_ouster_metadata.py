import json
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader

# ================= CONFIGURATION =================
BAG_PATH = 'garage_v2.2.0.bag'  # ROS bag path
TOPIC_METADATA = '/ouster/metadata'

def main():
    bag_path_obj = Path(BAG_PATH)
    if not bag_path_obj.exists():
        print(f"Error: {BAG_PATH} not found.")
        return

    print(f"Reading {BAG_PATH}...")
    
    metadata_str = None
    with AnyReader([bag_path_obj]) as reader:
        connections = [x for x in reader.connections if x.topic == TOPIC_METADATA]
        for conn, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            metadata_str = msg.data
            break 

    if metadata_str is None:
        print("Error: No metadata found.")
        return

    # Parse JSON
    try:
        data = json.loads(metadata_str)
    except Exception as e:
        print(f"Error parsing JSON: {e}")
        return

    print("=" * 50)
    print("OUSTER RESOLUTION REPORT")
    print("=" * 50)

    # 1. ELEVATION (Vertical) RESOLUTION
    # Located in: beam_intrinsics -> beam_altitude_angles
    if 'beam_intrinsics' in data and 'beam_altitude_angles' in data['beam_intrinsics']:
        angles = np.array(data['beam_intrinsics']['beam_altitude_angles'])
        
        # Sort angles to ensure differences are calculated correctly
        # (Ouster sometimes lists them top-to-bottom or interleaved)
        angles_sorted = np.sort(angles)
        
        # Calculate spacing between every beam
        diffs = np.diff(angles_sorted)
        
        # Ouster beams are often non-uniform (wider at edges, tighter in center).
        # We report the Median (typical) and the Min/Max range.
        res_median = np.median(diffs)
        fov = angles_sorted[-1] - angles_sorted[0]
        
        print(f"--- Vertical (Elevation) ---")
        print(f"Number of Beams:     {len(angles)}")
        print(f"Total Vertical FOV:  {fov:.2f} degrees")
        print(f"Median Resolution:   {res_median:.4f} degrees")
        print(f"Min/Max Spacing:     {np.min(diffs):.4f} / {np.max(diffs):.4f} degrees")
        
    else:
        print("Could not find 'beam_altitude_angles' in beam_intrinsics.")

    # 2. AZIMUTH (Horizontal) RESOLUTION
    # This is defined by the number of columns per rotation (1024, 2048, etc.)
    # Priority 1: Check 'lidar_data_format' -> 'columns_per_frame'
    # Priority 2: Check 'config_params' -> 'lidar_mode'
    
    cols = None
    
    # Try finding columns directly
    if 'lidar_data_format' in data and 'columns_per_frame' in data['lidar_data_format']:
        cols = data['lidar_data_format']['columns_per_frame']
        print(f"\n--- Horizontal (Azimuth) ---")
        print(f"Source:              lidar_data_format['columns_per_frame']")
        
    # Fallback to string parsing "1024x10"
    elif 'config_params' in data and 'lidar_mode' in data['config_params']:
        mode_str = data['config_params']['lidar_mode']
        try:
            cols = int(mode_str.split('x')[0])
            print(f"\n--- Horizontal (Azimuth) ---")
            print(f"Source:              config_params['lidar_mode'] ({mode_str})")
        except:
            pass

    if cols:
        az_res = 360.0 / float(cols)
        print(f"Horizontal Columns:  {cols}")
        print(f"Azimuth Resolution:  {az_res:.4f} degrees")
    else:
        print("\nCould not determine Horizontal Resolution (columns_per_frame not found).")

if __name__ == "__main__":
    main()