# %% Original notebook cell 1
import os
import csv
import json
import glob
import numpy as np
import open3d as o3d
from tqdm import tqdm

# ============================================================
# Parameters
# ============================================================

LIDAR_DIR = "lidar_frames"
LIDAR_TS_CSV = os.path.join(LIDAR_DIR, "timestamps.csv")
ODOM_CSV = "odom.csv"
TF_STATIC_JSON = "tf_static.json"

FRAME_ACCUM = 6              # old 20 was too large for ~10 Hz lidar
VOXEL_SIZE = 0.15
MAP_VOXEL = 0.08

MIN_RANGE = 1.0
MAX_RANGE = 40.0
MIN_Z = -2.0
MAX_Z = 2.5

ICP_COARSE_DIST = 1.5
ICP_FINE_DIST = 0.5

ADD_TO_MAP_EVERY = 1         # add every chunk to global map
USE_ODOM_PRIOR = True
USE_TF_STATIC = True

# ============================================================
# Basic transforms
# ============================================================

def quat_to_rot(qx, qy, qz, qw):
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    qw, qx, qy, qz = q / n

    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ], dtype=np.float64)
    return R

def make_T(tx, ty, tz, qx, qy, qz, qw):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = [tx, ty, tz]
    return T

def inv_T(T):
    T_inv = np.eye(4, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

# ============================================================
# Load exported files
# ============================================================

files = sorted(glob.glob(os.path.join(LIDAR_DIR, "frame_*.npy")))
print("Total raw lidar frames:", len(files))

# lidar timestamps
lidar_ts = []
with open(LIDAR_TS_CSV, "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        lidar_ts.append({
            "frame_id": int(row["frame_id"]),
            "bag_timestamp_ns": int(row["bag_timestamp_ns"]),
            "header_stamp_sec": int(row["header_stamp_sec"]),
            "header_stamp_nanosec": int(row["header_stamp_nanosec"]),
            "frame_id_msg": row["frame_id_msg"]
        })

print("Loaded lidar timestamp rows:", len(lidar_ts))

# odom
odom_rows = []
with open(ODOM_CSV, "r", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        odom_rows.append({
            "bag_timestamp_ns": int(row["bag_timestamp_ns"]),
            "header_stamp_sec": int(row["header_stamp_sec"]),
            "header_stamp_nanosec": int(row["header_stamp_nanosec"]),
            "frame_id": row["frame_id"],
            "child_frame_id": row["child_frame_id"],
            "T": make_T(
                float(row["pos_x"]), float(row["pos_y"]), float(row["pos_z"]),
                float(row["quat_x"]), float(row["quat_y"]), float(row["quat_z"]), float(row["quat_w"])
            )
        })

print("Loaded odom rows:", len(odom_rows))

# tf_static
tf_static = []
if USE_TF_STATIC and os.path.exists(TF_STATIC_JSON):
    with open(TF_STATIC_JSON, "r") as f:
        tf_static = json.load(f)
print("Loaded tf_static records:", len(tf_static))

# ============================================================
# Find lidar extrinsic from tf_static if possible
# We try to find transform between odom child frame and lidar frame.
# If not found, fallback to identity.
# ============================================================

lidar_frame_name = lidar_ts[0]["frame_id_msg"] if len(lidar_ts) > 0 else "livox_frame"
odom_child_name = odom_rows[0]["child_frame_id"] if len(odom_rows) > 0 else ""

def tf_record_to_T(rec):
    t = rec["translation"]
    r = rec["rotation"]
    return make_T(t["x"], t["y"], t["z"], r["x"], r["y"], r["z"], r["w"])

T_child_to_lidar = np.eye(4, dtype=np.float64)
found_extrinsic = False

if USE_TF_STATIC and len(tf_static) > 0 and odom_child_name != "":
    for rec in tf_static:
        parent = rec["header_frame_id"]
        child = rec["child_frame_id"]

        if parent == odom_child_name and child == lidar_frame_name:
            T_child_to_lidar = tf_record_to_T(rec)
            found_extrinsic = True
            print(f"Found tf_static: {odom_child_name} -> {lidar_frame_name}")
            break

        if parent == lidar_frame_name and child == odom_child_name:
            T_child_to_lidar = inv_T(tf_record_to_T(rec))
            found_extrinsic = True
            print(f"Found tf_static: {lidar_frame_name} -> {odom_child_name}, inverted")
            break

if not found_extrinsic:
    print("[WARN] No static transform found between odom child frame and lidar frame. Use identity.")

# ============================================================
# Timestamp matching
# ============================================================

odom_times = np.array([r["bag_timestamp_ns"] for r in odom_rows], dtype=np.int64)

def nearest_odom_pose(ts_ns):
    idx = np.searchsorted(odom_times, ts_ns)
    if idx <= 0:
        return odom_rows[0]["T"]
    if idx >= len(odom_rows):
        return odom_rows[-1]["T"]

    t0 = odom_times[idx - 1]
    t1 = odom_times[idx]

    if abs(ts_ns - t0) <= abs(ts_ns - t1):
        return odom_rows[idx - 1]["T"]
    else:
        return odom_rows[idx]["T"]

# ============================================================
# Point cloud preprocessing
# ============================================================

def filter_points(pts):
    xyz = pts[:, :3]

    r = np.linalg.norm(xyz, axis=1)
    mask = (r >= MIN_RANGE) & (r <= MAX_RANGE)
    mask &= (xyz[:, 2] >= MIN_Z) & (xyz[:, 2] <= MAX_Z)

    xyz = xyz[mask]
    return xyz

def load_and_accumulate_chunk(start_idx):
    pts_all = []
    ts_all = []

    for k in range(FRAME_ACCUM):
        idx = start_idx + k
        if idx >= len(files):
            break

        pts = np.load(files[idx])
        xyz = filter_points(pts)
        if xyz.shape[0] > 0:
            pts_all.append(xyz)

        ts_all.append(lidar_ts[idx]["bag_timestamp_ns"])

    if len(pts_all) == 0:
        return None, None

    pts_all = np.vstack(pts_all)
    chunk_ts = int(np.median(ts_all))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_all)
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)

    if len(pcd.points) < 30:
        return None, chunk_ts

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=VOXEL_SIZE * 2.5, max_nn=30)
    )
    return pcd, chunk_ts

# ============================================================
# ICP helpers
# ============================================================

def multi_scale_icp(source, target, init_T):
    # coarse
    reg1 = o3d.pipelines.registration.registration_icp(
        source, target,
        ICP_COARSE_DIST,
        init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    # fine
    reg2 = o3d.pipelines.registration.registration_icp(
        source, target,
        ICP_FINE_DIST,
        reg1.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )

    return reg2

# ============================================================
# Main SLAM
# ============================================================

global_map = o3d.geometry.PointCloud()
poses_global = []
chunk_times = []

prev_local = None
prev_global_pose = None
prev_odom_pose_lidar = None

n_chunks = 0

for start_idx in tqdm(range(0, len(files), FRAME_ACCUM)):
    pcd_local, chunk_ts = load_and_accumulate_chunk(start_idx)

    if pcd_local is None:
        continue

    # odom pose of robot child frame in odom/world
    if USE_ODOM_PRIOR and len(odom_rows) > 0:
        T_world_child = nearest_odom_pose(chunk_ts)
        T_world_lidar_odom = T_world_child @ T_child_to_lidar
    else:
        T_world_lidar_odom = np.eye(4, dtype=np.float64)

    if prev_local is None:
        # first chunk
        if USE_ODOM_PRIOR:
            T_world_lidar = T_world_lidar_odom.copy()
        else:
            T_world_lidar = np.eye(4, dtype=np.float64)

        pcd_global = o3d.geometry.PointCloud(pcd_local)
        pcd_global.transform(T_world_lidar)
        global_map += pcd_global

        prev_local = pcd_local
        prev_global_pose = T_world_lidar.copy()
        prev_odom_pose_lidar = T_world_lidar_odom.copy()

        poses_global.append(T_world_lidar.copy())
        chunk_times.append(chunk_ts)
        n_chunks += 1
        continue

    # source=current local, target=prev local
    if USE_ODOM_PRIOR:
        init_rel = inv_T(prev_odom_pose_lidar) @ T_world_lidar_odom
    else:
        init_rel = np.eye(4, dtype=np.float64)

    reg = multi_scale_icp(pcd_local, prev_local, init_rel)

    # current global pose = prev_global_pose @ (current->prev)
    T_world_lidar = prev_global_pose @ reg.transformation

    pcd_global = o3d.geometry.PointCloud(pcd_local)
    pcd_global.transform(T_world_lidar)

    if (n_chunks % ADD_TO_MAP_EVERY) == 0:
        global_map += pcd_global

    poses_global.append(T_world_lidar.copy())
    chunk_times.append(chunk_ts)

    prev_local = pcd_local
    prev_global_pose = T_world_lidar.copy()
    prev_odom_pose_lidar = T_world_lidar_odom.copy()
    n_chunks += 1

# ============================================================
# Save
# ============================================================

print("Final downsampling map...")
global_map = global_map.voxel_down_sample(MAP_VOXEL)

o3d.io.write_point_cloud("map_icp_odom.ply", global_map)
np.save("trajectory_icp_odom.npy", np.array(poses_global))
np.save("trajectory_icp_chunk_times.npy", np.array(chunk_times, dtype=np.int64))

print("Done.")
print("Saved: map_icp_odom.ply")
print("Saved: trajectory_icp_odom.npy")
print("Chunks:", len(poses_global))
print("Map points:", np.asarray(global_map.points).shape[0])

# %% Original notebook cell 2
import open3d as o3d

pcd = o3d.io.read_point_cloud("map_icp_odom.ply")
print(pcd)
o3d.visualization.draw_geometries([pcd])

# %% Original notebook cell 3
import numpy as np
import matplotlib.pyplot as plt

traj = np.load("trajectory_icp_odom.npy")
xyz = traj[:, :3, 3]

print("trajectory shape:", traj.shape)

plt.figure(figsize=(8, 6))
plt.plot(xyz[:, 0], xyz[:, 1], '-')
plt.axis('equal')
plt.xlabel("x")
plt.ylabel("y")
plt.title("ICP + Odom trajectory (top view)")
plt.grid(True)
plt.show()

# %% Original notebook cell 4

