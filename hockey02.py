import cv2
import numpy as np
#Descomentar en caso de querer bajar la velocidad del video
import time

# Read the video file
#video = cv2.VideoCapture(0)
video = cv2.VideoCapture("Hockey01.mp4")

# Create a window to display the video
cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

# Initialize the circle detector parameters
params = cv2.SimpleBlobDetector_Params()
params.filterByArea = True
params.minArea = 1000
params.maxArea = 5000
detector = cv2.SimpleBlobDetector_create(params)

# Initialize variables for tracking the circle
prev_center = (0,0)
prev_radius = 0
robty_proj = 600
redBajo1 = np.array([0, 200, 20], np.uint8)
redAlto1 = np.array([8, 255, 255], np.uint8)
redBajo2=np.array([175, 200, 20], np.uint8)
redAlto2=np.array([179, 255, 255], np.uint8)

# Loop through each frame of the video
while True:
    # Read the frame
    ret, frame = video.read()
    cv2.rectangle(frame, (220,95), (1015,610), (255,0,0), 2)
    # If there are no more frames, break out of the loop
    if not ret:
        break

    # Convert the frame to grayscale
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply a Gaussian blur to the frame to reduce noise
    blurred = cv2.GaussianBlur(frame, (25, 25), 0)
    frameHSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)
    edges = cv2.Canny(maskRed,50,200)
    cv2.imshow("Blurred", edges)
    # Detect the circle in the frame
    keypoints = detector.detect(edges)
    cv2.circle(frame, (300, int(robty_proj)), 40, (255, 0, 0), 2)
    # If a circle is detected, draw it on the frame
    if len(keypoints) > 0:
        center = (int(keypoints[0].pt[0]), int(keypoints[0].pt[1]))
        radius = int(keypoints[0].size / 2)
        cv2.circle(frame, center, radius, (0, 255, 0), 2)
        # ecuacion recta
        x1 = int(prev_center[0])
        y1 = int(prev_center[1])
        x2 = int(center[0])
        y2 = int(center[1])
        robtx = 300
        robty = int(((y2-y1)/(x2-x1+0.000001))*(robtx-x1)+y1)
        if robty > 610 :
            robty  = 600
        if robty < 95:
            robty = 90
        robt_pos = (int(robtx), int(robty))
        print(f"proyeccion:{robty}")
        
        if prev_center > center:
            cv2.line(frame, robt_pos, center, (255,0,0), 2)
            cv2.circle(frame, robt_pos, radius, (0, 255, 0), 2)
        if robty_proj < robty:
            robty_proj = robty_proj + 10
        if robty_proj > robty:
            robty_proj = robty_proj -10
        # Store the current circle's position and size for the next frame
        prev_center = center
        prev_radius = radius
        #imprimir posision del centro para debug
        #print(f"Centro:{center}")
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
