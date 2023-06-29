import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d

FOTOSTOTALES=1

for foto in range(1,FOTOSTOTALES+1):

    # Inicializar la cámara RealSense 
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)

    # Obtener el sensor de profundidad
    depth_sensor = profile.get_device().first_depth_sensor()

    # Configurar el rango de profundidad entre 0 y 1 metro


    # Esperar a que la cámara se estabilice
    for i in range(30):
        pipeline.wait_for_frames()


    # Obtener los datos de la cámara
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Convertir los datos en matrices numpy
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Crear un objeto PointCloud
    pc = rs.pointcloud()

    # Generar una nube de puntos a partir de los datos de la cámara
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)

    # Convertir la nube de puntos en una matriz numpy
    vtx = np.asanyarray(points.get_vertices())

    
    print(vtx)
    cv2.waitKey()
    np.savetxt("realsense\\nube.txt",vtx)
    # Crear un objeto PointCloud de open3d a partir de la matriz de puntos


    pcd = o3d.io.read_point_cloud("realsense\\nube.txt", format="xyz")
    o3d.io.write_point_cloud("realsense\\output.ply", pcd)
    # Detener la cámara
    pipeline.stop()
    cv2.destroyAllWindows()

    pcd = o3d.io.read_point_cloud("realsense\\output.ply")
    vtx = np.asarray(pcd.points)
    distances = np.linalg.norm(vtx, axis=1)
    mask = distances <= 1.0
    new_pcd = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(vtx[mask]))
    # Guarda la nueva nube de puntos en un archivo PLY
    o3d.io.write_point_cloud("realsense\\newOutput"+str(foto)+".ply", new_pcd)
    cv2.putText(color_image, "Espacio para nueva foto ({}/{})".format(foto, FOTOSTOTALES), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    # Mostrar la imagen de color y la nube de puntos 
    cv2.imshow("Color Image", color_image)
    if cv2.waitKey(1) == ord(' '):
        continue
    
"""
N = numero de imagenes totales
INTELCAMERA=inicializar_camara()
para todas las fortos en N
    miestras no puse espacio
        mostrar_imagen_camara(INTELCAMERA)
    deep_image=capturar_imagen_profundidad(INTELCAMERA)
    deep_image_recortada=pass_through_filter(deep_image,x,y,z)
    deep_image_filtrada=statistical_outlier_removal(deep_image_recortada,nb_neighbors,std_ratio)
    guardar_imagen(deep_image_filtrada)
N ← número de imágenes totales
INTELCAMERA ← inicializarCamara()
for all fotos en N do
while no puse espacio do
mostrarImagenCamara(IN T ELCAM ERA)
end while
deepImage ← capturarimagenprof undidad(INTELCAMERA)
deepImageFiltrada ← statisticalOutlierRemoval(deepImageRecortada, nbneighbors, stdratio)
guardarimagen(deepImageFiltrada)
end for




"""