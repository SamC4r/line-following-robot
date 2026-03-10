import cv2
import numpy as np
import time

# =========================
# CONFIG
# =========================

# Put your phone stream URL here
VIDEO_URL = "http://192.168.2.242:8080/video"

BASE_SPEED = 100
KP = 0.20
MAX_SPEED = 170
MIN_SPEED = 0

LOWER_BLUE = np.array([100, 100, 60])
UPPER_BLUE = np.array([140, 255, 255])

MIN_CONTOUR_AREA = 400
ROI_START_RATIO = 0.65
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
        LAST_COMMAND = cmd


def print_stop():
    print_command(0, 0)


def main():
    print("[INFO] Opening phone video stream...")
    cap = cv2.VideoCapture(VIDEO_URL)

    if not cap.isOpened():
        print("[ERROR] Could not open video stream.")
        return

    prev_time = time.time()

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

            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Threshold blue
            mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

            # Clean noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Region of interest
            roi_y = int(h * ROI_START_RATIO)
            roi = mask[roi_y:h, :]

            cnt = largest_contour(roi)
            line_found = False
            cx_global = None
            cy_global = None

            if cnt is not None:
                area = cv2.contourArea(cnt)
                if area > MIN_CONTOUR_AREA:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cx_global = cx
                        cy_global = cy + roi_y
                        line_found = True

                        cv2.drawContours(frame[roi_y:h, :], [cnt], -1, (0, 255, 0), 2)
                        cv2.circle(frame, (cx_global, cy_global), 8, (0, 0, 255), -1)

            # Draw center line and ROI
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
                cv2.putText(frame, f"Error: {error}", (10, 60),
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

            cv2.imshow("Robot View", frame)
            cv2.imshow("Blue Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        print("[INFO] Closing.")
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
