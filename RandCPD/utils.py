import numpy as np
import open3d as o3d
from os import walk
import copy

def read_ply(source,target):
    source = o3d.io.read_point_cloud(source,format='ply')
    target = o3d.io.read_point_cloud(target,format='ply')
    #o3d.visualization.draw_geometries([source])
    #o3d.visualization.draw_geometries([target])
    return source,target

def draw_registration_result(source,target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])
    
def draw_point_cloud(point_cloud):
    point_cloud_temp = copy.deepcopy(point_cloud)
    point_cloud_temp.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([point_cloud_temp])

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def preprocess_point_cloud_no_voxelize(pcd, voxel_size):
    pcd_down = pcd

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def get_points(poin_cloud):
    return np.asarray(poin_cloud.points)

def join_points_clouds(source, target):
    target=get_points(target)
    source=get_points(source)
    return np.concatenate((target,source), axis=0)

def points_to_point_cloud(points):
    return o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(points))

def save_point_cloud(point_cloud,name="RandCPD\\result.ply"):
    if isinstance(point_cloud,np.ndarray):
        point_cloud=points_to_point_cloud(point_cloud)
    o3d.io.write_point_cloud(name, point_cloud)
