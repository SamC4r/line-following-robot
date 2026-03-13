#MAS INFO: https://medium.com/@staytechrich/computer-vision-001-color-detection-with-opencv-58426c880449


import cv2
import numpy as np
import time
import serial



VIDEO_URL = "http://192.168.4.1:81/stream"


SERIAL_PORT = "COM3" #"/dev/ttyUSB0"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

BASE_SPEED = 100
KP = 1
MAX_SPEED = 300
MIN_SPEED = 0

# MEJORA: Rango HSV ajustado. V sube hasta 255 para tolerar reflejos fuertes. 
# S baja a 40 para no perder la línea en zonas de iluminación irregular.
LOWER_BLUE = np.array([85, 40, 40]) # EN HSV
UPPER_BLUE = np.array([115, 255, 255]) # EN HSV

LOWER_RED_1 = np.array([0, 100, 80])
UPPER_RED_1 = np.array([10, 255, 255])

LOWER_RED_2 = np.array([170, 100, 80])
UPPER_RED_2 = np.array([179, 255, 255])


MIN_CONTOUR_AREA = 400
ROI_START_RATIO = 0.5   # ROI => REgion of interest
FRAME_WIDTH = 480

# Avoid printing the same command every single frame unless it changes enough
LAST_COMMAND = None


def clamp(x, low, high):
    return max(low, min(high, x))


def largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def print_command(left_speed, right_speed):
    global LAST_COMMAND
    cmd = f"M,{left_speed},{right_speed}"
    if cmd != LAST_COMMAND:
        print(cmd)
        ser.write((cmd + "\n").encode())
        LAST_COMMAND = cmd


def print_stop():
    print_command(0, 0)


def main():
    print("[INFO] Opening phone video stream...")
    cap = cv2.VideoCapture(VIDEO_URL,cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("[ERROR] Could not open video stream.")
        return

    prev_time = time.time()
    
    # MEJORA: Kernel más grande (9x9) para "soldar" mejor los trozos de línea 
    # que se puedan separar por los brillos del suelo.
    kernel = np.ones((9, 9), np.uint8)

    kernel_red_open = np.ones((3, 3), np.uint8)
    kernel_red_close = np.ones((5, 5), np.uint8)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Failed to read frame.")
                print_stop()
                continue

            # Resize for speed
            h0, w0 = frame.shape[:2]
            scale = FRAME_WIDTH / float(w0)
            frame = cv2.resize(frame, (FRAME_WIDTH, int(h0 * scale)))

            h, w = frame.shape[:2]
            center_x = w // 2

            # OPTIMIZACIÓN: Definimos y recortamos el ROI *antes* de procesar el color.
            # Esto ahorra CPU porque OpenCV procesará la mitad de los píxeles de la imagen.
            roi_y = int(h * ROI_START_RATIO)
            roi_frame = frame[roi_y:h, :]

            # MEJORA: Desenfoque gaussiano sobre el ROI para suavizar la textura del suelo.
            blurred_roi = cv2.GaussianBlur(roi_frame, (9, 9), 0)

            # Convertir a HSV (solo procesamos el ROI)
            hsv_roi = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)

            # MEJORA: Para el rojo no ROI. TODO EL FRAME
            blurred_full_frame = cv2.GaussianBlur(frame, (9, 9), 0)
            hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)


            #######################
            ### PARA AZUL ########
            ######################



            # Threshold blue
            mask = cv2.inRange(hsv_roi, LOWER_BLUE, UPPER_BLUE)


            # QUitar ruido usando el kernel 9x9
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)






            
            #######################
            ### PARA ROJO ########
            ######################

            # Dos rangos para rojo
            mask1 = cv2.inRange(hsv_full, LOWER_RED_1, UPPER_RED_1)
            mask2 = cv2.inRange(hsv_full, LOWER_RED_2, UPPER_RED_2)

            mask_red = cv2.bitwise_or(mask1, mask2)

            # Limpieza

            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel_red_open)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel_red_close)  


            # Buscar contorno en la máscara que ahora ya es solo del tamaño del ROI
            cnt = largest_contour(mask)
            line_found = False
            cx_global = None
            cy_global = None

            red_contour = largest_contour(mask_red)

            # Draw the contours on the original image
            if red_contour is not None:
                contour = red_contour
                if cv2.contourArea(contour) > MIN_CONTOUR_AREA:
                    #print("FOUND")
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)



            if cnt is not None:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        # Centroides relativos al ROI
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Convertir a coordenadas globales para dibujar en el frame original
                        cx_global = cx
                        cy_global = cy + roi_y 
                        line_found = True

                        # Desplazar las coordenadas del contorno para dibujarlo bien en la imagen completa
                        cnt_shifted = cnt + np.array([[0, roi_y]])
                        cv2.drawContours(frame, [cnt_shifted], -1, (0, 255, 0), 3)
                        cv2.circle(frame, (cx_global, cy_global), 8, (0, 0, 255), -1)

            # Draw center line and ROI boundaries
            cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 2)
            cv2.rectangle(frame, (0, roi_y), (w, h), (255, 255, 0), 2)

            if line_found:
                error = cx_global - center_x
                turn = KP * error

                left_speed = int(BASE_SPEED + turn)
                right_speed = int(BASE_SPEED - turn)

                left_speed = clamp(left_speed, MIN_SPEED, MAX_SPEED)
                right_speed = clamp(right_speed, MIN_SPEED, MAX_SPEED)

                print_command(left_speed, right_speed)

                cv2.putText(frame, "LINE FOUND", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Error: {error:.1f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"L:{left_speed} R:{right_speed}", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                print_stop()
                cv2.putText(frame, "LINE LOST", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            now = time.time()
            fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Mostrar ventanas
            cv2.imshow("Robot View", frame)
            cv2.imshow("Blue Mask (ROI solo)", mask) 
            cv2.imshow("Red Mask", mask_red)

    finally:
        print("[INFO] Closing.")
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()