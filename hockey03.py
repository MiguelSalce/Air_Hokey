# importar librerias
import cv2
import numpy as np

# Se abre la camara (1 por ser camara externa)
video = cv2.VideoCapture(1)


# Se crea una pantalla para visualiazcion
cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

# Parametros de deteccion del circulo
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 8000
params.maxArea = 12000
detector = cv2.SimpleBlobDetector_create(params)

# Variables iniciales
prev_center = (0,0)
prev_radius = 0
Ry_p = 40
Rx_p = 50
Rx = 50
Ry = 40
Ry1 = 39
Top = 20
Left = 25
Bot = 600
Right = 460
Blue = (255, 0, 0)
Green = (0, 255, 0)
Radio = 0.2
Rpm = 0
V1=0

# Valores del filto de rojos
redBajo1 = np.array([0, 110, 50], np.uint8)
redAlto1 = np.array([15, 255, 255], np.uint8)
redBajo2=np.array([165, 110, 50], np.uint8)
redAlto2=np.array([179, 255, 255], np.uint8)

# Loop para cada frame capturado
while True:
    # Lee la camara
    ret, frame = video.read()
    # Cierra el programa si hay error de lectura
    if not ret:
        break
    # Tratado de la imagen para la deteccion del circulo
    blurred = cv2.GaussianBlur(frame, (25, 25), 0) # Filtro gaussiano
    frameHSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # Convertir de RGV a HSV
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1) # Filtro inferior de rojos
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2) # Filtro superior de rojos
    maskRed = cv2.add(maskRed1, maskRed2) # Union de ambos filtros
    edges = cv2.Canny(maskRed,50,200) # Aplica filtro Canny para dejar lo contornos
    cv2.imshow("Blurred", edges) # Proyeccion del Canny para debug
    # Deteccion del circulo en el frame procesado
    keypoints = detector.detect(edges)
    # Se delimita el area de juego
    cv2.rectangle(frame, (Top,Left), (Bot,Right), Blue, 2)
    # Si se detecta el circulo se dibuja el contorno y se empieza la prediccion
    if len(keypoints) > 0:
        center = (int(keypoints[0].pt[0]), int(keypoints[0].pt[1]))
        radius = int(keypoints[0].size / 2)
        cv2.circle(frame, center, radius, Green, 2)
        # Si el circulo se mueve hacia el Robot se proyecta la trayectoria
        if prev_center > center:
            # Valores para la ecuacion de la recta
            x1 = int(prev_center[0])
            y1 = int(prev_center[1])
            x2 = int(center[0])
            y2 = int(center[1])
            # Ecuacion de la recta proyectada
            Ry_p = int(((y2-y1)/(x2-x1+0.000001))*(Rx-x1)+y1)
            # Si se proyecta una trayectoria fuera de la meza se limita el movimiento del robot
            if Ry_p > Bot:
                Ry_p  = 460
            if Ry_p < Top:
                Ry_p = 25
            Rp_pos = (int(Rx_p), int(Ry_p))
            #print(f"proyeccion:{Ry_p}")
            # Proyeccion de la trayectoria
            cv2.line(frame, Rp_pos, center, Blue, 2)
            # Projeccion del robot en x fija
            cv2.circle(frame, Rp_pos, radius, Green, 2)
            # Se inicia el control
            Ry_e = Ry_p - Ry # Error de posicion
            Dp=np.sqrt((Ry_p-y2)**2+(x2-Rx_p)**2) # Distancia puck
            Vp=(np.sqrt((x2-x1)**2+(y2-y1)**2))/0.03 # Velocidad del puck
            Tp=Dp/Vp # Tiempo para la interseccion
            Kp1= 0.7 # Ganancia del propocional de posicion
            Vta = Ry_e*Kp1/Tp # Velocidad Target
            Va=(Ry-Ry1)/0.03 # Velocidad actual del Robot
            Ve = Vta-Va # Velocidad error del robot
            Kp2 = 1.5 # Ganancia del proporcional de Velocidad
            Rpm = (Ve*Kp2)*Radio # Rpm para el contolador
            if Ry_p==x2:
                IV = 0
            IntV = IntV+(0.03*V1+0.5*0.03*(Va-V1))
            IV = (Ve*Kp2) + IntV
            Ry = Ry + (IV)*0.03 # Posicion del robot
            Ry1=Ry
            V1=Va
        else:
            Rpm = 0
            IntV = 0
        # Se guardan los valores de centro y radio 
        prev_center = center
        prev_radius = radius
    if len(keypoints) == 0:
        Rpm = 0
        IntV = 0
    # Se dibuja el circulo donde se encuentra el robot
    cv2.circle(frame, (int(Rx),int(Ry)), 25, Blue, 2)
    print(f"proyeccion:{Rpm}")
# Enseñar frame en la pantalla
    cv2.imshow("Video", frame)
#Descomentar en caso de querer bajar la velocidad del video
    #time.sleep(0.1)
# Cerrar programa al presionar q
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Clean up
video.release()
cv2.destroyAllWindows()
