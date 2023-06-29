import cpd
import rancac2
import open3d as o3d
import copy
import numpy as np
 

X="C:\\Users\\Usuario\\Desktop\\tfg-fran\\blenderData\\blensor\\RANGEsCAN\\limpio\\ImageToStl.com_male00001.ply"
Y="C:\\Users\\Usuario\\Desktop\\tfg-fran\\blenderData\\blensor\\RANGEsCAN\\limpio\\ImageToStl.com_male00003.ply"

print("ransac")
source,target, voxel_size=rancac2.prepareData(X,Y,voxel_size=0.055)
rancac2.draw_registration_result(source,target)

source_down, source_fpfh = rancac2.preprocess_point_cloud(source, voxel_size)
target_down, target_fpfh = rancac2.preprocess_point_cloud(target, voxel_size)
#source_down.transform(transformacion-1)

result_ransac = rancac2.execute_global_registration(source_down, target_down,
                                            source_fpfh, target_fpfh,
                                            voxel_size)
rancac2.draw_registration_result_2(source_down, target_down, result_ransac.transformation)
source_down.transform(result_ransac.transformation)
# transformacion-1=result_ransac.transformation * transformacion-1
target=np.asarray(target_down.points)
source=np.asarray(source_down.points)
print(source)

nube=cpd.cpd_registration(target,source)
print(type(nube))
print(nube)
nube_o3d = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(nube))
o3d.visualization.draw_geometries([nube_o3d])
o3d.io.write_point_cloud("RandCPD\\result.ply", nube_o3d)






