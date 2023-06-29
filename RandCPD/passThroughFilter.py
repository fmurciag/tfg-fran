import open3d as o3d
import numpy as np


point_cloud=o3d.io.read_point_cloud("RandCPD\\result.ply",format='ply')
 
axis_min = 0.5
axis_max = 1.5

points = np.asarray(point_cloud.points)
indices = np.where((points[:, 0] >= axis_min) & (points[:, 0] <= axis_max))[0]
filtered_point_cloud = point_cloud.select_by_index(indices)


o3d.visualization.draw_geometries([filtered_point_cloud])