import cv2
import numpy as np
import time
import socket
import threading

car = socket.socket()
car.connect(("192.168.4.1", 100))

def receiver():
    buf = ""
    while True:
        try:
            buf += car.recv(1024).decode()
            while '{' in buf and '}' in buf:
                start = buf.index('{')
                end   = buf.index('}') + 1
                msg   = buf[start:end]
                buf   = buf[end:]
                if msg == "{Heartbeat}":
                    car.send(b"{Heartbeat}")
        except:
            break

threading.Thread(target=receiver, daemon=True).start()

VIDEO_URL = "http://192.168.4.1:81/stream"

BASE_SPEED = 70
MAX_SPEED  = 200
MIN_SPEED  = 50  

# PD tuning
kp = 0.4
kd = 1
ki = 0.003
ki_max = 100
integral = 0

# Level 1: adaptive speed
CURVE_BRAKE = 0.8   # how aggressively to slow down on curves (higher = slower on curves)
MIN_CURVE_SPEED = 50  # minimum speed when cornering

LOWER_BLUE  = np.array([85,  40,  40])
UPPER_BLUE  = np.array([115, 255, 255])

LOWER_RED_1 = np.array([0,   100, 80])
UPPER_RED_1 = np.array([10,  255, 255])
LOWER_RED_2 = np.array([170, 100, 80])
UPPER_RED_2 = np.array([179, 255, 255])

MIN_CONTOUR_AREA = 400
ROI_START_RATIO  = 0.5
FRAME_WIDTH      = 480

SEARCH_SPEED   = 70
SEARCH_TIMEOUT = 3.0

# ── State 
LAST_COMMAND   = None
last_error     = 0
prev_error     = 0
line_lost_time = None

# ── Helpers ───────────────────────────────────────────────────
def clamp(x, low, high):
    return max(low, min(high, x))

def largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)

def send_command(left, right):
    global LAST_COMMAND
    left  = int(clamp(left,  -MAX_SPEED, MAX_SPEED))
    right = int(clamp(right, -MAX_SPEED, MAX_SPEED))
    cmd = f"M,{left},{right}"
    if cmd != LAST_COMMAND:
        print(cmd)
        car.send(("{" + cmd + "}").encode())
        LAST_COMMAND = cmd

def stop():
    send_command(0, 0)

def main():
    global last_error, prev_error, line_lost_time,integral

    print("[INFO] Opening stream...")
    cap = cv2.VideoCapture(VIDEO_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[ERROR] Could not open stream.")
        return

    prev_time        = time.time()
    kernel           = np.ones((9, 9), np.uint8)
    kernel_red_open  = np.ones((3, 3), np.uint8)
    kernel_red_close = np.ones((5, 5), np.uint8)

    time.sleep(1)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame read failed.")
                stop()
                continue

            # ── Resize 
            h0, w0 = frame.shape[:2]
            frame   = cv2.resize(frame, (FRAME_WIDTH, int(h0 * FRAME_WIDTH / w0)))
            h, w    = frame.shape[:2]
            center_x = w // 2

            # ── ROI
            roi_y     = int(h * ROI_START_RATIO)
            roi_frame = frame[roi_y:h, :]

            # ── Blue mask 
            blurred_roi = cv2.GaussianBlur(roi_frame, (9, 9), 0)
            hsv_roi     = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)
            mask        = cv2.inRange(hsv_roi, LOWER_BLUE, UPPER_BLUE)
            mask        = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
            mask        = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # ── Red mask 
            blurred_full = cv2.GaussianBlur(frame, (9, 9), 0)
            hsv_full     = cv2.cvtColor(blurred_full, cv2.COLOR_BGR2HSV)
            mask_red     = cv2.bitwise_or( cv2.inRange(hsv_full, LOWER_RED_1, UPPER_RED_1), cv2.inRange(hsv_full, LOWER_RED_2, UPPER_RED_2))
            mask_red     = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN,  kernel_red_open)
            mask_red     = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel_red_close)

            # ── Red line → intersection 
            red_cnt      = largest_contour(mask_red)
            red_detected = (red_cnt is not None and
                            cv2.contourArea(red_cnt) > MIN_CONTOUR_AREA * 3)

            if red_detected:
                cv2.drawContours(frame, [red_cnt], -1, (0, 165, 255), 3)
                cv2.putText(frame, "RED LINE - STOP/TURN", (10, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                stop()

            # ── Blue line tracking ────────────────────────────
            cnt        = largest_contour(mask)
            line_found = False

            if cnt is not None and cv2.contourArea(cnt) > MIN_CONTOUR_AREA:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx        = int(M["m10"] / M["m00"])
                    cy        = int(M["m01"] / M["m00"])
                    cx_global = cx
                    cy_global = cy + roi_y
                    line_found = True

                    last_error     = cx_global - center_x
                    line_lost_time = None

                    cnt_shifted = cnt + np.array([[0, roi_y]])
                    cv2.drawContours(frame, [cnt_shifted], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cx_global, cy_global), 8, (0, 0, 255), -1)

            # Draw guides
            cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 2)
            cv2.rectangle(frame, (0, roi_y), (w, h), (255, 255, 0), 2)

            if line_found and not red_detected:
                #PID control ALGORITHM

                derror     =   last_error - prev_error
                integral  +=   last_error
                integral   =   clamp(integral, -ki_max, ki_max)

                turn       =   (kp * last_error) + (kd * derror) + (ki * integral)
                prev_error =   last_error

                curve_sharpness = abs(derror)
                dynamic_speed   = BASE_SPEED - (curve_sharpness * CURVE_BRAKE)
                dynamic_speed   = clamp(dynamic_speed, MIN_CURVE_SPEED, MAX_SPEED)

                left_speed  = dynamic_speed + turn
                right_speed = dynamic_speed - turn

                send_command(left_speed, right_speed)

                cv2.putText(frame, "LINE FOUND",                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Error: {last_error}  D: {derror:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Turn: {turn:.1f}  Speed: {dynamic_speed:.0f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            elif not red_detected:
                # ── Recovery ──────────────────────────────────
                if line_lost_time is None:
                    line_lost_time = time.time()


                integral = 0
                elapsed = time.time() - line_lost_time

                if elapsed < SEARCH_TIMEOUT:
                    if last_error > 0:
                        send_command(SEARCH_SPEED, -SEARCH_SPEED)
                        cv2.putText(frame, "SEARCHING → RIGHT", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    else:
                        send_command(-SEARCH_SPEED, SEARCH_SPEED)
                        cv2.putText(frame, "SEARCHING ← LEFT",  (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                    cv2.putText(frame, f"Timeout: {SEARCH_TIMEOUT - elapsed:.1f}s", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                else:
                    stop()
                    cv2.putText(frame, "LINE LOST - STOPPED", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            fps       = 1.0 / max(time.time() - prev_time, 1e-6)
            prev_time = time.time()
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("Robot View", frame)
            cv2.imshow("Blue Mask",  mask)
            cv2.imshow("Red Mask",   mask_red)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop()
        cap.release()
        cv2.destroyAllWindows()
        car.close()
        print("[INFO] Done.")

if __name__ == "__main__":
    main()