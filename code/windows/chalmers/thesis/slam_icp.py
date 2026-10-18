# %% Original notebook cell 1
import numpy as np
import open3d as o3d
import glob
from tqdm import tqdm

# -------------------------
# parameters
# -------------------------

FRAME_ACCUM = 20
VOXEL_SIZE = 0.2
ICP_DIST = 1.5

# -------------------------
# load frame list
# -------------------------

files = sorted(glob.glob("lidar_frames/frame_*.npy"))

print("Total raw frames:", len(files))

# -------------------------
# helper function
# -------------------------

def load_and_accumulate(start_idx):

    pts_all = []

    for k in range(FRAME_ACCUM):

        if start_idx + k >= len(files):
            break

        pts = np.load(files[start_idx + k])[:, :3]
        pts_all.append(pts)

    pts_all = np.vstack(pts_all)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_all)

    return pcd


# -------------------------
# SLAM
# -------------------------

global_map = o3d.geometry.PointCloud()

poses = []

T = np.eye(4)

prev = None

for i in tqdm(range(0, len(files), FRAME_ACCUM)):

    pcd = load_and_accumulate(i)

    pcd = pcd.voxel_down_sample(VOXEL_SIZE)

    if prev is None:

        prev = pcd
        global_map += pcd
        poses.append(T.copy())
        continue

    # ICP
    result = o3d.pipelines.registration.registration_icp(
        pcd,
        prev,
        ICP_DIST,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    T = T @ result.transformation

    pcd.transform(T)

    global_map += pcd

    poses.append(T.copy())

    prev = pcd

# -------------------------
# save results
# -------------------------

print("Saving map...")

global_map = global_map.voxel_down_sample(0.1)

o3d.io.write_point_cloud("map.ply", global_map)

np.save("trajectory.npy", np.array(poses))

print("Finished")

# %% Original notebook cell 2
import open3d as o3d

pcd = o3d.io.read_point_cloud("map.ply")

o3d.visualization.draw_geometries([pcd])

# %% Original notebook cell 3
import numpy as np

traj = np.load("trajectory.npy")

xyz = traj[:, :3, 3]

print(xyz.shape)

# %% Original notebook cell 4

