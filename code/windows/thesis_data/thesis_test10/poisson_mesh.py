# %% Original notebook cell 1
import numpy as np
import open3d as o3d

pcd_path = "laser_map.pcd"

# 1. 读取点云
pcd = o3d.io.read_point_cloud(pcd_path)
print("原始点云:", pcd)

# 2. 下采样
pcd = pcd.voxel_down_sample(voxel_size=0.08)
print("下采样后:", pcd)

# 3. 统计去噪
pcd, _ = pcd.remove_statistical_outlier(
    nb_neighbors=20,
    std_ratio=1.2
)

# 4. 半径去噪
pcd, _ = pcd.remove_radius_outlier(
    nb_points=8,
    radius=0.35
)
print("去噪后:", pcd)

# 5. 法向量
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.6,
        max_nn=40
    )
)

# 让法向量朝外更一致
pcd.orient_normals_consistent_tangent_plane(20)

# 6. Poisson 重建
print("开始 Poisson...")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd,
    depth=7
)

# 7. 根据密度裁剪
densities = np.asarray(densities)
keep = densities > np.quantile(densities, 0.12)   # 去掉最差 12%
mesh.remove_vertices_by_mask(~keep)

# 8. 清理小碎片
triangle_clusters, cluster_n_triangles, cluster_area = mesh.cluster_connected_triangles()
triangle_clusters = np.asarray(triangle_clusters)
cluster_n_triangles = np.asarray(cluster_n_triangles)

# 只保留较大的连通块
triangles_to_remove = cluster_n_triangles[triangle_clusters] < 200
mesh.remove_triangles_by_mask(triangles_to_remove)
mesh.remove_unreferenced_vertices()

# 9. 法向量 + 上色
mesh.compute_vertex_normals()
mesh.paint_uniform_color([0.7, 0.7, 0.75])  # 改成浅灰，不会一片黑

# 10. 保存
mesh_path = "laser_map_poisson_mesh.ply"
o3d.io.write_triangle_mesh(mesh_path, mesh)
print("已保存:", mesh_path)

# 11. 显示
o3d.visualization.draw_geometries(
    [mesh],
    mesh_show_back_face=True
)

# %% Original notebook cell 2

