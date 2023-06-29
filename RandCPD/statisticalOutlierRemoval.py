import numpy as np
import open3d as o3d


point_cloud=o3d.io.read_point_cloud("RandCPD\\result.ply",format='ply')
o3d.visualization.draw_geometries([point_cloud, point_cloud])


nb_neighbors = 20  # Número de vecinos considerados
std_ratio = 2.0  # Umbral de desviación estándar


cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors, std_ratio)


point_cloud_cleaned = point_cloud.select_by_index(ind)


o3d.visualization.draw_geometries([point_cloud, point_cloud_cleaned])