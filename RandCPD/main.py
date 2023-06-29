import icp
import rancac2 as ransac
import cpd
import utils
import numpy as np
import open3d as o3d
import os  
import copy
from tqdm import tqdm

VIEW=True
LOGS=False
DIR="RandCPD\\d"
PREVIOUSTRANSFORMATION=np.identity(4)
VOXEL_SIZE=0.005#0.055 #a mas grande menos puntos 


def get_data(directory):
    
    F = []
    for (dirpath, dirnames, filenames) in os.walk(directory):
        F.extend(filenames)
        break
    return F

def prepare_data(source,target,first=False):
    source,target=utils.read_ply(source,target)
    source_down, source_fpfh = utils.preprocess_point_cloud(source, VOXEL_SIZE)
    if first:
        target_down, target_fpfh = utils.preprocess_point_cloud(target, VOXEL_SIZE)
    else:
        target_down, target_fpfh = utils.preprocess_point_cloud_no_voxelize(target, VOXEL_SIZE)
    if VIEW: utils.draw_registration_result(source_down,target_down)
    return source_down, target_down, source_fpfh, target_fpfh

def run_ransac(source,target,source_fpfh,target_fpfh, previous_transformation=None):
    result_ransac = ransac.execute_global_registration(source, target,
                                                source_fpfh, target_fpfh,
                                                VOXEL_SIZE)
    transformation=result_ransac.transformation
    source_tranform=source.transform(result_ransac.transformation)
    if VIEW: utils.draw_registration_result(source=source_tranform,target=target)
    return source_tranform, transformation

def run_cpd(source,target, previous_transformation=None):
    source_=utils.get_points(source)
    target_=utils.get_points(target)
    source_tranform_, transformation=cpd.cpd_registration2(source_, target_)
    source_tranform=utils.points_to_point_cloud(source_tranform_)
    if VIEW: utils.draw_registration_result(source=source_tranform,target=target)
    return source_tranform, transformation

def run_icp(source,target, previous_transformation=None):
    source_tranform, transformation=icp.run_icp(source,target)
    if VIEW: utils.draw_registration_result(source=source_tranform,target=target)
    return source_tranform, transformation

def main():
    global PREVIOUSTRANSFORMATION
    files = get_data(DIR)
    source_down = None
    target_down = None
    source_fpfh = None
    target_fpfh = None
    for i in tqdm(range(len(files))):
   
        if i == 0:
            source = os.path.join(DIR, files[1])
            target = os.path.join(DIR, files[0])
            if LOGS:print("prepare data1")
            source_down, target_down, source_fpfh, target_fpfh = prepare_data(source, target, first=True)

        if i == 1:
            continue

        if i > 1:
            source = os.path.join(DIR, files[i])
            target = "RandCPD\\result.ply"
            if LOGS:print("prepare data2")
            source_down, target_down, source_fpfh, target_fpfh = prepare_data(source, target)

        
        if LOGS:print("apli previous transformation")
        source_down.transform(PREVIOUSTRANSFORMATION)
        if LOGS:print("run ransac")
        #source_tranform, transformation = run_ransac(source_down, target_down, source_fpfh, target_fpfh)
        if LOGS:print("run cpd")
        #source_tranform, _ = run_cpd(source_tranform, target_down)
        source_tranform,_=run_icp(source_down,target_down)
        source_tranform, _= run_cpd(source_tranform, target_down)
    
        if LOGS:print("update previous transformation")
        

        #PREVIOUSTRANSFORMATION = np.dot(PREVIOUSTRANSFORMATION, transformation)

        result = utils.join_points_clouds(source=source_tranform, target=target_down)
        if LOGS:print("save")
        utils.save_point_cloud(result)

if __name__ == "__main__":
    main()

"""
VIEW = habilitar visualizaciones (bool)
LOGS = habilitar logs (bool)
DIR = Directorio de datos (text)
PREVIOUSTRANSFORMATION=matriz de identidad (int)[4,4]
VOXEL_SIZE = tamaño del voxel (int)

ficheros=obtener_vistas(DIR)
para todas las vistas :
    si primera iteracion:
        source = ficheros[0]
        target = ficheros[1]
        source_down, target_down, source_feature, target_feature=preparar_datos(source,target)
    resto de iteraciones:
        source = ficheros[i]
        target = resultado iteracion anterior
        source_down, target_down, source_feature, target_feature=preparar_datos(source,target)

    source_down.aplicar_transformacion(PREVIOUSTRANSFORMATION)

    source_tranform, transformacion=ransac(source, target,source_feature,target_feature)

    source_tranform, transformacion=ICP(source, target)
    
    source_tranform, transformacion=CPD(source, target)

    resultado = juntar_nubes_de_puntos(source_tranform,target_down)

    PREVIOUSTRANSFORMATION=PREVIOUSTRANSFORMATION * transformacion

    joinTransform(ransac,icp,cpd)

guardar_nube(resultado)

"""