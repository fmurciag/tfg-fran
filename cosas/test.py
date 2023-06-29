import open3d as o3d
import numpy as np
import copy

MAX_ITERATIONS=100
THRESHOLD=0.05

def run_icp_ply(source, target):
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, THRESHOLD, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=MAX_ITERATIONS))
        
    # Aplicar la transformación resultante a la nube de puntos source
    source_transformed = source.transform(reg_p2p.transformation)
    
    
    return source_transformed, reg_p2p.transformation

    
def draw_registration_result_2(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])




source_cloud = o3d.io.read_point_cloud("bun000.ply")
target_cloud = o3d.io.read_point_cloud("bun045.ply")
draw_registration_result_2(source_cloud, target_cloud)

sourceTrans,trans=run_icp_ply(source_cloud,target_cloud)

draw_registration_result_2(sourceTrans, target_cloud)

