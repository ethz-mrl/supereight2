import os
import csv
import struct
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader

# ================= CONFIGURATION =================
BAG_PATH = 'garage_v2.2.0.bag'  # ROS bag path
OUTPUT_DIR = 'scoutdi_dataset'

# Topics
TOPIC_ODOM = '/mavros/odometry/to_px4'
TOPIC_LIDAR = '/ouster/points'
TOPIC_UT_MEASUREMENT = '/api/ut_measurement'
TOPIC_TF_STATIC = '/tf_static'
TOPIC_TF = '/tf'

# Frames for Extrinsics
FRAME_BASE = 'base_link'
FRAME_LIDAR = 'os_sensor'     # Static
FRAME_UT_BASE = 'ut_probe_base' # Intermediate
FRAME_UT_FACE = 'ut_probe_face' # Dynamic

# ================= MATH HELPER =================

class Transform:
    def __init__(self, p=None, q=None):
        self.p = np.array(p if p is not None else [0.0, 0.0, 0.0])
        self.q = np.array(q if q is not None else [0.0, 0.0, 0.0, 1.0]) # [x, y, z, w]

    @staticmethod
    def from_msg(transform_msg):
        """Creates a Transform object from a ROS geometry_msgs/Transform"""
        t = transform_msg.translation
        r = transform_msg.rotation
        return Transform([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])

    def __mul__(self, other):
        """Combines two transforms: self * other (Parent->Child * Child->Grandchild)"""
        # Rotation of the other position by self quaternion
        p_rotated = self.rotate_vector(other.p)
        p_new = self.p + p_rotated
        
        # Quaternion multiplication
        q_new = self.quat_multiply(self.q, other.q)
        return Transform(p_new, q_new)

    def rotate_vector(self, v):
        """Rotates vector v by quaternion self.q"""
        q = self.q
        # Formula: v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + q_w * v)
        q_vec = q[:3]
        q_w = q[3]
        t = 2.0 * np.cross(q_vec, v)
        return v + q_w * t + np.cross(q_vec, t)

    def quat_multiply(self, q1, q2):
        """Multiplies two quaternions [x, y, z, w]"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])
    
    def __str__(self):
        return (f"Translation: x={self.p[0]:.6f}, y={self.p[1]:.6f}, z={self.p[2]:.6f}\n"
                f"Rotation:    x={self.q[0]:.6f}, y={self.q[1]:.6f}, z={self.q[2]:.6f}, w={self.q[3]:.6f}")

# ================= PARSING HELPERS =================

def ensure_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

def to_nanosec(sec, nanosec):
    return np.uint64(sec) * np.uint64(1_000_000_000) + np.uint64(nanosec)

def get_dtype(datatype):
    # 1:INT8, 2:UINT8, 3:INT16, 4:UINT16, 5:INT32, 6:UINT32, 7:FLOAT32, 8:FLOAT64
    mapping = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}
    return mapping.get(datatype, 'f')

def parse_point_cloud(msg):
    fields = {f.name: f for f in msg.fields}
    if 'x' not in fields: return []

    off_x, off_y, off_z = fields['x'].offset, fields['y'].offset, fields['z'].offset
    has_intensity = 'intensity' in fields
    off_i = fields['intensity'].offset if has_intensity else 0
    
    data = msg.data if isinstance(msg.data, (bytes, bytearray)) else msg.data.tobytes()
    point_step = msg.point_step
    num_points = msg.width * msg.height
    
    fmt_x = '<' + get_dtype(fields['x'].datatype)
    fmt_y = '<' + get_dtype(fields['y'].datatype)
    fmt_z = '<' + get_dtype(fields['z'].datatype)
    fmt_i = '<' + get_dtype(fields['intensity'].datatype) if has_intensity else None

    points = []
    for i in range(num_points):
        start = i * point_step
        x = struct.unpack_from(fmt_x, data, start + off_x)[0]
        y = struct.unpack_from(fmt_y, data, start + off_y)[0]
        z = struct.unpack_from(fmt_z, data, start + off_z)[0]
        intensity = struct.unpack_from(fmt_i, data, start + off_i)[0] if has_intensity else 0
        points.append((x, y, z, intensity))
    return points

# ================= MAIN =================

def main():
    ensure_dir(OUTPUT_DIR)
    bag_path_obj = Path(BAG_PATH)
    if not bag_path_obj.exists():
        print(f"Error: {BAG_PATH} not found.")
        return

    # --- STEP 1: Scan for Static Transforms (/tf_static) ---
    print("Pass 1: extracting static transforms...")
    
    T_base_lidar = None
    T_base_utbase = None
    
    with AnyReader([bag_path_obj]) as reader:
        connections = [x for x in reader.connections if x.topic == TOPIC_TF_STATIC]
        for conn, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            for tf in msg.transforms:
                parent = tf.header.frame_id
                child = tf.child_frame_id
                
                if parent == FRAME_BASE and child == FRAME_LIDAR:
                    T_base_lidar = Transform.from_msg(tf.transform)
                    print(f"  Found Static LiDAR: {parent} -> {child}")
                
                elif parent == FRAME_BASE and child == FRAME_UT_BASE:
                    T_base_utbase = Transform.from_msg(tf.transform)
                    print(f"  Found Static UT Base: {parent} -> {child}")

    # Check if we found what we need
    if T_base_lidar is None:
        print(f"WARNING: Static transform {FRAME_BASE}->{FRAME_LIDAR} not found in {TOPIC_TF_STATIC}")
    if T_base_utbase is None:
        print(f"WARNING: Static transform {FRAME_BASE}->{FRAME_UT_BASE} not found in {TOPIC_TF_STATIC}")

    # --- Save Static LiDAR Extrinsics ---
    if T_base_lidar:
        with open(os.path.join(OUTPUT_DIR, 'extrinsics_lidar.txt'), 'w') as f:
            f.write(f"Parent: {FRAME_BASE}\nChild: {FRAME_LIDAR}\n{T_base_lidar}\n")

    # --- STEP 2: Process All Topics (Dynamic) ---
    print("Pass 2: Processing dynamic topics...")
    
    # Open CSV Writers
    f_traj = open(os.path.join(OUTPUT_DIR, 'trajectory.csv'), 'w', newline='')
    f_lidar = open(os.path.join(OUTPUT_DIR, 'lidar.csv'), 'w', newline='')
    f_ut_meas = open(os.path.join(OUTPUT_DIR, 'ut_measurement.csv'), 'w', newline='')
    f_ut_pose = open(os.path.join(OUTPUT_DIR, 'ut_pose_dynamic.csv'), 'w', newline='')

    w_traj = csv.writer(f_traj)
    w_lidar = csv.writer(f_lidar)
    w_ut_meas = csv.writer(f_ut_meas)
    w_ut_pose = csv.writer(f_ut_pose)

    # Headers
    w_traj.writerow(['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    w_lidar.writerow(['timestamp', 'x', 'y', 'z', 'intensity'])
    w_ut_meas.writerow(['timestamp', 'thickness_m'])
    w_ut_pose.writerow(['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])

    count_lidar = 0
    count_ut_pose = 0

    with AnyReader([bag_path_obj]) as reader:
        connections = [x for x in reader.connections if x.topic in 
                       [TOPIC_ODOM, TOPIC_LIDAR, TOPIC_UT_MEASUREMENT, TOPIC_TF]]
        
        for conn, ts, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, conn.msgtype)

            # 1. Trajectory
            if conn.topic == TOPIC_ODOM:
                ts_ns = to_nanosec(msg.header.stamp.sec, msg.header.stamp.nanosec)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                w_traj.writerow([ts_ns, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

            # 2. LiDAR
            elif conn.topic == TOPIC_LIDAR:
                ts_ns = to_nanosec(msg.header.stamp.sec, msg.header.stamp.nanosec)
                points = parse_point_cloud(msg)
                for pt in points:
                    w_lidar.writerow([ts_ns, pt[0], pt[1], pt[2], pt[3]])
                count_lidar += 1

            # 3. UT Measurement
            elif conn.topic == TOPIC_UT_MEASUREMENT:
                ts_ns = to_nanosec(msg.header.stamp.sec, msg.header.stamp.nanosec)
                val = getattr(msg, 'thickness_m', 0.0)
                w_ut_meas.writerow([ts_ns, val])

            # 4. UT Dynamic Pose (Transform Chaining)
            elif conn.topic == TOPIC_TF:
                if T_base_utbase is None: continue # Cannot compute if static part is missing
                
                if hasattr(msg, 'transforms'):
                    for tf in msg.transforms:
                        # Look for ut_probe_base -> ut_probe_face
                        if tf.header.frame_id == FRAME_UT_BASE and tf.child_frame_id == FRAME_UT_FACE:
                            ts_ns = to_nanosec(tf.header.stamp.sec, tf.header.stamp.nanosec)
                            
                            # Create transform for this dynamic step
                            T_utbase_utface = Transform.from_msg(tf.transform)
                            
                            # CHAIN: (base -> ut_base) * (ut_base -> ut_face) = (base -> ut_face)
                            T_base_utface = T_base_utbase * T_utbase_utface
                            
                            w_ut_pose.writerow([
                                ts_ns,
                                T_base_utface.p[0], T_base_utface.p[1], T_base_utface.p[2],
                                T_base_utface.q[0], T_base_utface.q[1], T_base_utface.q[2], T_base_utface.q[3]
                            ])
                            count_ut_pose += 1

    # Cleanup
    f_traj.close()
    f_lidar.close()
    f_ut_meas.close()
    f_ut_pose.close()

    print("\nExtraction Complete.")
    print(f"  - LiDAR Frames processed: {count_lidar}")
    print(f"  - UT Poses calculated:    {count_ut_pose}")
    print(f"  - Data saved to:          {OUTPUT_DIR}/")

if __name__ == "__main__":
    main()