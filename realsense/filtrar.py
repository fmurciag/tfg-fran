import open3d as o3d
import numpy as np

# Carga la nube de puntos desde un archivo PLY
pcd = o3d.io.read_point_cloud("realsense\\output.ply")

vtx = np.asarray(pcd.points)
distances = np.linalg.norm(vtx, axis=1)
mask = distances <= 1.0
new_pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(vtx[mask]))
# Guarda la nueva nube de puntos en un archivo PLY
o3d.io.write_point_cloud("realsense\\newOutput.ply", new_pcd)