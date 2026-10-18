# %% Original notebook cell 1
import open3d as o3d

pcd = o3d.io.read_point_cloud("laser_map.pcd")

# 下采样
pcd = pcd.voxel_down_sample(0.05)

# 统计去噪
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# 半径去噪（重点）
#pcd, _ = pcd.remove_radius_outlier(nb_points=10, radius=0.3)

# 法向量
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30)
)

o3d.visualization.draw_geometries([pcd])

# %% Original notebook cell 2
import open3d as o3d

pcd = o3d.io.read_point_cloud("laser_map.pcd")

print(pcd)  # 查看点数

o3d.visualization.draw_geometries([pcd])

# %% Original notebook cell 3
import open3d as o3d

pcd = o3d.io.read_point_cloud("rtab_cloud.ply")

print(pcd)  # 查看点数

o3d.visualization.draw_geometries([pcd])

# %% Original notebook cell 4


# %% Original notebook cell 5

