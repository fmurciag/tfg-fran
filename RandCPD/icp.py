import open3d as o3d
import numpy as np
import utils
MAX_ITERATION=1000000
THRESHOLD=0.05
MAX_ITERATIONS=100
THRESHOLD=0.05

def run_icp(source, target):
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, THRESHOLD, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=MAX_ITERATIONS))
        
    # Aplicar la transformación resultante a la nube de puntos source
    source_transformed = source.transform(reg_p2p.transformation)
    
    
    return source_transformed, reg_p2p.transformation

