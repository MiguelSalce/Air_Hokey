# importar librerias
import cv2
import numpy as np
import serial
import time

# Se abre la camara (1 por ser camara externa)
video = cv2.VideoCapture(0)
# inicio de la comunicacion serial
ser = serial.Serial("COM10", 74880)
# Se crea una pantalla para visualiazcion
cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

# Parametros de deteccion del circulo rojo
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 1000
params.maxArea = 1500
detector = cv2.SimpleBlobDetector_create(params)

# Parametros de deteccion del circulo Azul
paramsBlue = cv2.SimpleBlobDetector_Params()
paramsBlue.filterByArea = True
paramsBlue.minArea = 1
paramsBlue.maxArea = 1000
detectorBlue = cv2.SimpleBlobDetector_create(paramsBlue)

# Variables iniciales del control
prev_center = (0,0) # Centro del frame previo
Ry_p = 40 # Posicion y predecida
Rx_p = 50 # Posicion x predecida
Rx = 50 # Posicion x 
Ry = 0 # Posicion y
Ry1 = 39 # Posicion y del frame previo
Top = 20 # Limite superior
Left = 25 # Limite izquierdo
Bot = 600 # Limite inferior
Right = 460 # Limite Derecho
Blue = (255, 0, 0) #Color azul
Green = (0, 255, 0) #Color verde
Red = (0, 0, 255) #Color rojo
Radio = 1.7 # Radio del motor
Kp1= 0.7 # Ganancia del propocional de posicion
Ry_e = 0 #Error de posicion en y
Pul = 0 #Pulsos para el motor


# Valores del filtro de rojos
redBajo1 = np.array([0, 110, 50], np.uint8)
redAlto1 = np.array([15, 255, 255], np.uint8)
redBajo2=np.array([165, 110, 50], np.uint8)
redAlto2=np.array([179, 255, 255], np.uint8)

# Valores de filtro de azules
BlueBajo = np.array([105, 150, 50], np.uint8)
BlueAlto = np.array([135, 255, 255], np.uint8)

# Loop para cada frame capturado
while True:
    # Lee la camara
    ret, frame = video.read()
    # Cierra el programa si hay error de lectura
    if not ret:
        print("No camara detectada")
        break
    # Procesamiento del Frame 
    blurred = cv2.GaussianBlur(frame, (25, 25), 0) # Filtro gaussiano
    frameHSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # Convertidor de RGV a HSV
    
    # Deteccion del circulo rojo en el frame procesado
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1) # Filtro inferior de rojos
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2) # Filtro superior de rojos
    maskRed = cv2.add(maskRed1, maskRed2) # Union de ambos filtros
    edges = cv2.Canny(maskRed,50,200) # Aplica filtro Canny para dejar lo contornos
    keypoints = detector.detect(edges) # Deteccion del circulo rojo
    
    # Deteccion del circulo azul en el frame procesado
    maskBlue = cv2.inRange(frameHSV, BlueBajo, BlueAlto) # Filtro inferior de rojos
    edgesBlue = cv2.Canny(maskBlue,50,200) # Aplica filtro Canny para dejar lo contornos
    keypointsBlue = detectorBlue.detect(edgesBlue) # Deteccion del circulo azul
   
    # Se delimita el area de juego
    cv2.rectangle(frame, (Top,Left), (Bot,Right), Blue, 2)
    
    # Si se detecta un circulo azul se proyecta en la imagen
    if len(keypointsBlue) > 0:
        centerBlue = (int(keypointsBlue[0].pt[0]), int(keypointsBlue[0].pt[1]))
        radiusBlue = int(keypointsBlue[0].size / 2)
        Ry = int(centerBlue[1])
        # Se dibuja el circulo donde se encuentra el robot
        cv2.circle(frame, centerBlue, 25, Blue, 2)
        
    # Si se detecta el circulo rojo se dibuja el contorno
    if len(keypoints) > 0:
        en = 1
        center = (int(keypoints[0].pt[0]), int(keypoints[0].pt[1]))
        radius = int(keypoints[0].size / 2)
        cv2.circle(frame, center, radius, Red, 2)
        
        # Si el circulo se mueve hacia el Robot se proyecta la trayectoria
        if prev_center[0] > (center[0]+5):
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
            # Se guarda el valor del centro predecido
            Rp_pos = (int(Rx_p), int(Ry_p))
            # Proyeccion de la trayectoria
            cv2.line(frame, Rp_pos, center, Blue, 2)
            # Projeccion del robot en x fija
            cv2.circle(frame, Rp_pos, radius, Green, 2)
            # Se inicia el control
            Ry_e = Ry_p - Ry # Error de posicion
            Px = Ry_e*Kp1 # pixels para llegar a la posicion target
            Pul = ((Px)/-0.1655) # conversion pixels a pulsos
            if Ry==y2: # Si el puck choca con el robot los pulsos son 0 
                Pul = 0
        else: # los pulsos son igual a 0 cuando el puck se aleja
            Pul = 0
        # Se guardan los valores de centro
        prev_center = center
    # Si no se detecta puck los pulsos son igual a 0
    if len(keypoints) == 0: 
        Pul = 0
    
    # Se envian los pulsos por la comunicacnon serial
    ESP32 = str(int(Pul)) #Se guarda el valor de pulso para ser enviado a la ESP32
    ser.write(ESP32.encode('ascii')) # Se codifica a ascii
    time.sleep(1) # segundo de delay para el movimiento del motor
    print(f"Pulsos:{ESP32}")
    
    # Enseñar frame en la pantalla
    cv2.imshow("Video", frame)
    
    # Cerrar programa al presionar q
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Clean up
video.release()
cv2.destroyAllWindows()
ser.close()
