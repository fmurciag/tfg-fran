import open3d as o3d
import copy
import numpy as np


#fucnion para dibujar
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


#crear nubes de puntos 
demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud("bun000.ply",format='ply')
target = o3d.io.read_point_cloud("bun045.ply",format='ply')
threshold = 0.02
trans_init = np.asarray([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
draw_registration_result(source, target, trans_init)


#aplicar ICP
reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
print(reg_p2p)

print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)

reg_p2p = o3d.pipelines.registration.registration_icp(
    source, target, threshold, trans_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
print(reg_p2p)
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)