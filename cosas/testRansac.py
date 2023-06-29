from RandCPD import utils
import open3d as o3d
import numpy as np


def execute_global_registration(source_down, target_down, source_surf, target_surf, voxel_size):
    distance_threshold = voxel_size
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_surf, target_surf, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999999))

    inlier_rmse = result.inlier_rmse

    return inlier_rmse






def preprocess_point_cloud_fpfh(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def preprocess_point_cloud_surf(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    keypoints = o3d.keypoint.sift.detect_sift_keypoints(pcd_down, o3d.pybind.geometry.PointCloud.get_default_SIFT_params())
    descriptors = o3d.descriptor.sift.compute_sift_descriptor(pcd_down, keypoints, o3d.pybind.geometry.PointCloud.get_default_SIFT_params())

    return pcd_down, keypoints, descriptors

S,T=utils.read_ply("RandCPD\\dataCuerpo\\newOutput1.ply","RandCPD\\dataCuerpo\\newOutput2a.ply")
source,spurce_feature=preprocess_point_cloud_fpfh(S, 0.055)
target,target_feature=preprocess_point_cloud_fpfh(T, 0.055)

inlier_rmse=execute_global_registration(source, target,spurce_feature,target_feature,0.055)
print(inlier_rmse)