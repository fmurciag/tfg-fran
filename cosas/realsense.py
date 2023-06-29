import cv2
import time

# Configuración del video
ancho_video = 640
alto_video = 480
fps_video = 25
duracion_video = 20  # Duración deseada del video en segundos

# Crear objeto de captura de video
captura = cv2.VideoCapture(1)
captura.set(cv2.CAP_PROP_FRAME_WIDTH, ancho_video)
captura.set(cv2.CAP_PROP_FRAME_HEIGHT, alto_video)

# Configuración del codec y el archivo de salida
codec = cv2.VideoWriter_fourcc(*'XVID')
archivo_salida = cv2.VideoWriter('video_salida.avi', codec, fps_video, (ancho_video, alto_video))

# Inicializar variables de tiempo y contador de imágenes
tiempo_inicial = time.time()
num_frames = 0

# Bucle de grabación
while True:
    # Capturar frame de la cámara
    ret, frame = captura.read()

    # Comprobar si se ha capturado correctamente el frame
    if not ret:
        break
    
    tiempo_actual = time.time()
    tiempo_transcurrido = tiempo_actual - tiempo_inicial
    # Superponer texto en el frame
    if int(tiempo_transcurrido) % 2 == 0:
        texto = 'Img capturada'
    else:
        texto = 'Gire'

    cv2.putText(frame, texto, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Mostrar el frame en una ventana
    cv2.imshow('Video', frame)

    # Guardar el frame en el archivo de salida
    archivo_salida.write(frame)

    # Comprobar si se ha alcanzado la duración deseada
    tiempo_actual = time.time()
    tiempo_transcurrido = tiempo_actual - tiempo_inicial
    if tiempo_transcurrido >= duracion_video:
        break

    # Incrementar el contador de frames
    num_frames += 1

    # Comprobar si se presiona la tecla 'q' para detener la grabación
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos
captura.release()
archivo_salida.release()
cv2.destroyAllWindows()