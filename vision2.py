import cv2
import numpy as np
import time
import socket
import threading
import math

car = socket.socket()
car.connect(("192.168.4.1", 100))


def receiver():
    buf = ""
    while True:
        try:
            buf += car.recv(1024).decode()
            while '{' in buf and '}' in buf:
                start = buf.index('{')
                end = buf.index('}') + 1
                msg = buf[start:end]
                buf = buf[end:]
                if msg == "{Heartbeat}":
                    car.send(b"{Heartbeat}")
        except:
            break


threading.Thread(target=receiver, daemon=True).start()

VIDEO_URL = "http://192.168.4.1:81/stream"

BASE_SPEED = 30
MAX_SPEED = 3 * BASE_SPEED
MIN_SPEED = 10

kp = 0.65
kd = 0.4
ki = 0.001
ki_max = 20
integral = 0

kp_turn = 0.25
kd_turn = 0.31
ki_turn = 0.0004

CURVE_BRAKE = 0.5
MIN_CURVE_SPEED = MIN_SPEED

LOWER_BLUE = np.array([80, 20, 100])
UPPER_BLUE = np.array([125, 255, 255])

LOWER_RED_1 = np.array([0, 60, 70])
UPPER_RED_1 = np.array([12, 255, 255])

LOWER_RED_2 = np.array([165, 60, 70])
UPPER_RED_2 = np.array([179, 255, 255])

MIN_CONTOUR_AREA = 400
ROI_START_RATIO = 0.2
FRAME_WIDTH = 480
SEARCH_SPEED = 2 * BASE_SPEED
SEARCH_TIMEOUT = 3.0

ZONE_PIXEL_RATIO = 0.01
INTERSECTION_CONFIRM_FRAMES = 5
INTERSECTION_COOLDOWN = 0.5

INTERSECTION_MAP = {
    (False, True,  False, True):  "STRAIGHT",
    (True,  True,  True,  False): "T-LR",
    (True,  True,  True,  True):  "X",
    (True,  True,  False, True):  "T-LEFT",
    (False, True,  True,  True):  "T-RIGHT",
    (False, True,  True,  False): "SHARP-RIGHT",
    (True,  True,  False, False): "SHARP-LEFT",

    (False, False, False, False): "NO LINE",
}

INTERSECTION_OPTIONS = {
    "STRAIGHT":       ["S"],
    "T-LR":           ["L", "R"],
    "X":              ["L", "R", "S"],
    "T-LEFT":         ["L", "S"],
    "T-RIGHT":        ["R", "S"],
    "SHARP-RIGHT":    ["R"],
    "SHARP-LEFT":     ["L"]

}

DEFAULT_DIRECTIONS = {
    "STRAIGHT":       "S",
    "T-LR":           "S",
    "X":              "S",
    "T-LEFT":         "S",
    "T-RIGHT":        "S",
    "SHARP-RIGHT":    "S",
    "SHARP-LEFT":     "S"
}

STATE_FOLLOWING = "FOLLOWING"
STATE_TURNING = "TURNING"

LAST_COMMAND = None
last_error = 0
prev_error = 0
line_lost_time = None

state = STATE_FOLLOWING
pending_direction = None
intersection_count = 0
last_intersection_time = 0.0
current_intersection = None
last_intersection_label = None
last_decision_source = ""

DEBUG_ARROW = True
ARROW_DEBUG_INTERVAL = 0.15
last_arrow_debug_time = 0.0


def debug_arrow(msg):
    global last_arrow_debug_time
    if not DEBUG_ARROW:
        return

    now = time.time()
    if now - last_arrow_debug_time >= ARROW_DEBUG_INTERVAL:
        print(f"[ARROW DEBUG] {msg}")
        last_arrow_debug_time = now

def clamp(x, low, high):
    return max(low, min(high, x))


def largest_contour(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def send_command(left, right):
    global LAST_COMMAND
    left = int(clamp(left, -MAX_SPEED, MAX_SPEED))
    right = int(clamp(right, -MAX_SPEED, MAX_SPEED))
    cmd = f"M,{left},{right}"
    if cmd != LAST_COMMAND:
        car.send(("{" + cmd + "}").encode())
        LAST_COMMAND = cmd


def stop():
    send_command(0, 0)


def detect_arrow_orientation(cnt):
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None, None, None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    pts = cnt.reshape(-1, 2)
    d = np.sum((pts - np.array([cx, cy])) ** 2, axis=1)
    tip = pts[np.argmax(d)]

    dx = int(tip[0] - cx)
    dy = int(tip[1] - cy)

    angle = math.degrees(math.atan2(dy, dx))

    debug_arrow(
        f"center=({cx},{cy}) tip=({tip[0]},{tip[1]}) dx={dx} dy={dy} angle={angle:.2f}"
    )

    return angle, (cx, cy), (tip[0], tip[1])

def classify_arrow_direction(angle):
    if angle is None:
        debug_arrow("angle is None")
        return None

    if -45 <= angle <= 45:
        direction = "L"
    elif angle >= 135 or angle <= -135:
        direction = "R"
    else:
        direction = "S"

    debug_arrow(f"classified angle={angle:.2f} as direction={direction}")
    return direction


def detect_arrow(frame, cnt):
    angle, center, tip = detect_arrow_orientation(cnt)
    arrow_direction = classify_arrow_direction(angle)
    print("AAAAA",arrow_direction)
    if angle is not None:
        cv2.circle(frame, center, 6, (255, 0, 0), -1)
        cv2.circle(frame, tip, 8, (0, 0, 255), -1)
        cv2.line(frame, center, tip, (0, 255, 255), 2)

        cv2.putText(
            frame,
            f"{angle:.1f} deg  ARROW: {arrow_direction}",
            (center[0] + 10, center[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

        debug_arrow(
            f"visualized arrow center={center} tip={tip} angle={angle:.2f} direction={arrow_direction}"
        )

    return arrow_direction


def detect_intersection(mask, frame, roi_y):
    fh, fw = frame.shape[:2]
    h, w = mask.shape[:2]
    third = w // 3

    zones = {
        "left": mask[:, 0:third],
        "center": mask[:, third:2 * third],
        "right": mask[:, 2 * third:w],
    }

    results = {}
    for name, zone in zones.items():
        results[name] = int(np.count_nonzero(zone)) / max(zone.size, 1) >= ZONE_PIXEL_RATIO

    left_active = results["left"]
    center_active = results["center"]
    right_active = results["right"]

    sq_x1 = max(0, 35 * w // 100)
    sq_x2 = max(0, 65 * w // 100)
    sq_y1 = max(0, 3 * fh // 10 - roi_y)
    sq_y2 = max(0, 5 * fh // 10 - roi_y)
    sq_roi = mask[sq_y1:sq_y2, sq_x1:sq_x2]

    sq_active = (
        sq_roi.size > 0 and
        int(np.count_nonzero(sq_roi)) / sq_roi.size >= ZONE_PIXEL_RATIO
    )

    label = INTERSECTION_MAP.get(
        (left_active, center_active, right_active, sq_active),
        f"UNKNOWN ({left_active},{center_active},{right_active},{sq_active})"
    )

    zone_colors = {"left": (255, 200, 0), "center": (0, 255, 200), "right": (200, 0, 255)}
    zone_starts = [0, third, 2 * third]
    zone_names = ["left", "center", "right"]
    zone_actives = [left_active, center_active, right_active]

    for i, (zname, x_start) in enumerate(zip(zone_names, zone_starts)):
        x_end = x_start + third if i < 2 else w
        color = zone_colors[zname]
        active = zone_actives[i]
        pt1, pt2 = (x_start, roi_y), (x_end - 1, fh - 1)

        if active:
            alpha = frame.copy()
            cv2.rectangle(alpha, pt1, pt2, color, -1)
            cv2.addWeighted(alpha, 0.12, frame, 0.88, 0, frame)

        cv2.rectangle(frame, pt1, pt2, color, 2 if active else 1)
        cv2.putText(
            frame,
            f"{'■' if active else '□'} {zname.upper()}",
            (x_start + 4, roi_y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
        )

    sq_color = (0, 255, 100) if sq_active else (100, 100, 100)
    cv2.rectangle(frame, (sq_x1, 3 * fh // 10), (sq_x2, 5 * fh // 10), sq_color, 2)
    cv2.putText(
        frame,
        f"{'■' if sq_active else '□'} SQ",
        (sq_x1 + 4, 3 * fh // 5 + 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        sq_color,
        1,
    )

    banner_color = (0, 255, 0) if label == "STRAIGHT" else (0, 165, 255)
    cv2.putText(
        frame,
        f"INTERSECTION: {label}",
        (10, fh - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        banner_color,
        2,
    )

    return label, left_active, center_active, right_active, sq_active


def choose_direction(intersection_label, arrow_direction, arrow_detected):
    allowed = INTERSECTION_OPTIONS.get(intersection_label)
    if not allowed:
        debug_arrow(f"intersection={intersection_label} has no allowed options")
        return None, ""

    debug_arrow(
        f"intersection={intersection_label} allowed={allowed} arrow_detected={arrow_detected} arrow_direction={arrow_direction}"
    )

    if arrow_detected and arrow_direction in allowed:
        debug_arrow(f"decision from arrow -> {arrow_direction}")
        return arrow_direction, "ARROW"

    default_direction = DEFAULT_DIRECTIONS[intersection_label]
    debug_arrow(f"decision from default -> {default_direction}")
    return default_direction, "DEFAULT"


def trigger_turn(direction, label, now, source):
    global state, pending_direction
    global intersection_count, last_intersection_time, current_intersection
    global integral, prev_error, last_error, last_decision_source

    intersection_count = 0
    current_intersection = label
    last_intersection_time = now
    pending_direction = direction
    last_decision_source = source
    integral = 0
    prev_error = 0
    last_error = 0
    state = STATE_TURNING


def main():
    global last_error, prev_error, line_lost_time, integral
    global state, pending_direction
    global intersection_count, last_intersection_time, current_intersection
    global last_intersection_label, last_decision_source

    print("[INFO] Opening stream...")
    cap = cv2.VideoCapture(VIDEO_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[ERROR] Could not open stream.")
        return

    prev_time = time.time()
    kernel = np.ones((9, 9), np.uint8)
    kernel_red_open = np.ones((3, 3), np.uint8)
    kernel_red_close = np.ones((5, 5), np.uint8)

    time.sleep(1)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Frame read failed.")
                stop()
                continue

            h0, w0 = frame.shape[:2]
            frame = cv2.resize(frame, (FRAME_WIDTH, int(h0 * FRAME_WIDTH / w0)))
            h, w = frame.shape[:2]
            center_x = w // 2
            third = w // 3

            roi_y = int(h * ROI_START_RATIO)
            roi_frame = frame[roi_y:h, :]

            blurred_roi = cv2.GaussianBlur(roi_frame, (9, 9), 0)
            hsv_roi = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_roi, LOWER_BLUE, UPPER_BLUE)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            blurred_full = cv2.GaussianBlur(frame, (9, 9), 0)
            hsv_full = cv2.cvtColor(blurred_full, cv2.COLOR_BGR2HSV)
            mask_red = cv2.bitwise_or(
                cv2.inRange(hsv_full, LOWER_RED_1, UPPER_RED_1),
                cv2.inRange(hsv_full, LOWER_RED_2, UPPER_RED_2)
            )
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel_red_open)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel_red_close)

            intersection_label, z_left, z_center, z_right, z_sq = detect_intersection(mask, frame, roi_y)

            if intersection_label not in ("STRAIGHT", "NO LINE") and not intersection_label.startswith("UNKNOWN"):
                last_intersection_label = intersection_label

            red_cnt = largest_contour(mask_red)
            red_detected = red_cnt is not None and cv2.contourArea(red_cnt) > MIN_CONTOUR_AREA
            arrow_direction = None

            if red_detected:
                cv2.drawContours(frame, [red_cnt], -1, (0, 165, 255), 3)
                cv2.putText(frame, "RED ARROW", (10, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                arrow_direction = detect_arrow(frame, red_cnt)

            if state == STATE_TURNING and pending_direction == "R":
                track_mask = mask[:, 2 * third:]
                x_offset = 2 * third
                dot_color = (0, 100, 255)
            elif state == STATE_TURNING and pending_direction == "L":
                track_mask = mask[:, :third]
                x_offset = 0
                dot_color = (0, 100, 255)
            else:
                track_mask = mask[:, third:2 * third]
                x_offset = third
                dot_color = (0, 0, 255)

            cnt_draw = largest_contour(mask)
            cnt_track = largest_contour(track_mask)
            line_found = False

            if cnt_track is not None and cv2.contourArea(cnt_track) > MIN_CONTOUR_AREA:
                M = cv2.moments(cnt_track)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + x_offset
                    cy = int(M["m01"] / M["m00"])
                    cx_global = cx
                    cy_global = cy + roi_y
                    line_found = True
                    last_error = cx_global - center_x
                    line_lost_time = None

                    if cnt_draw is not None:
                        cnt_shifted = cnt_draw + np.array([[0, roi_y]])
                        cv2.drawContours(frame, [cnt_shifted], -1, (0, 255, 0), 3)

                    cv2.circle(frame, (cx_global, cy_global), 8, dot_color, -1)

            cv2.line(frame, (third, roi_y), (third, h), (0, 200, 255), 1)
            cv2.line(frame, (2 * third, roi_y), (2 * third, h), (0, 200, 255), 1)
            cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 2)
            cv2.rectangle(frame, (0, roi_y), (w, h), (255, 255, 0), 2)

            now = time.time()

            if state == STATE_FOLLOWING:
                actionable = intersection_label in DEFAULT_DIRECTIONS
                cooldown_ok = (now - last_intersection_time) > INTERSECTION_COOLDOWN

                if actionable and cooldown_ok:
                    intersection_count += 1
                else:
                    if not actionable:
                        intersection_count = 0

                if intersection_count >= INTERSECTION_CONFIRM_FRAMES and cooldown_ok:
                    direction, source = choose_direction(intersection_label, arrow_direction, red_detected)
                    if direction is not None:
                        print(f"[AUTO] {intersection_label} -> {direction} ({source})")
                        trigger_turn(direction, intersection_label, now, source)

                elif line_found:
                    derror = last_error - prev_error
                    integral += last_error
                    integral = clamp(integral, -ki_max, ki_max)
                    turn = (kp * last_error) + (kd * derror) + (ki * integral)
                    prev_error = last_error

                    dynamic_speed = clamp(BASE_SPEED - abs(derror) * CURVE_BRAKE, MIN_CURVE_SPEED, MAX_SPEED)
                    send_command(dynamic_speed + turn, dynamic_speed - turn)

                    cv2.putText(frame, "LINE FOUND", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Error: {last_error}  D: {derror:.1f}", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Turn: {turn:.1f}  Speed: {dynamic_speed:.0f}", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                elif not red_detected:
                    if line_lost_time is None:
                        line_lost_time = now

                    integral = 0
                    elapsed = now - line_lost_time

                    if elapsed < SEARCH_TIMEOUT:
                        if last_intersection_label == "SHARP-LEFT":
                            send_command(-SEARCH_SPEED, SEARCH_SPEED)
                            cv2.putText(frame, "SEARCHING LEFT (SHARP-LEFT)", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        elif last_intersection_label == "SHARP-RIGHT":
                            send_command(SEARCH_SPEED, -SEARCH_SPEED)
                            cv2.putText(frame, "SEARCHING RIGHT (SHARP-RIGHT)", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        elif last_error > 0:
                            send_command(SEARCH_SPEED, -SEARCH_SPEED)
                            cv2.putText(frame, "SEARCHING RIGHT", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        else:
                            send_command(-SEARCH_SPEED, SEARCH_SPEED)
                            cv2.putText(frame, "SEARCHING LEFT", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                        cv2.putText(frame, f"Timeout: {SEARCH_TIMEOUT - elapsed:.1f}s", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    else:
                        stop()
                        cv2.putText(frame, "LINE LOST - STOP", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            elif state == STATE_TURNING:
                cv2.putText(frame, f"TURNING {pending_direction} ({last_decision_source})", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 0), 2)

                center_zone = mask[:, third:2 * third]
                center_ratio = np.count_nonzero(center_zone) / max(center_zone.size, 1)
                center_has_line = center_ratio >= ZONE_PIXEL_RATIO

                turn_done = False

                if pending_direction == "R":
                    side_zone = mask[:, 2 * third:]
                    side_ratio = np.count_nonzero(side_zone) / max(side_zone.size, 1)
                    side_has_line = side_ratio >= ZONE_PIXEL_RATIO
                    turn_done = center_has_line and not side_has_line

                elif pending_direction == "L":
                    side_zone = mask[:, :third]
                    side_ratio = np.count_nonzero(side_zone) / max(side_zone.size, 1)
                    side_has_line = side_ratio >= ZONE_PIXEL_RATIO
                    turn_done = center_has_line and not side_has_line

                else:
                    turn_done = center_has_line

                derror = last_error - prev_error
                integral += last_error
                integral = clamp(integral, -ki_max, ki_max)
                turn = (kp_turn * last_error) + (kd_turn * derror) + (ki_turn * integral)
                prev_error = last_error

                dynamic_speed = clamp(BASE_SPEED - abs(derror) * CURVE_BRAKE, MIN_CURVE_SPEED, MAX_SPEED)
                send_command(dynamic_speed + turn, dynamic_speed - turn)

                if turn_done:
                    print(f"[INFO] Turn complete -> follow")
                    integral = 0
                    prev_error = 0
                    last_intersection_time = time.time()
                    state = STATE_FOLLOWING

            if red_detected and arrow_direction is not None:
                cv2.putText(frame, f"ARROW DIR: {arrow_direction}", (10, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            fps = 1.0 / max(time.time() - prev_time, 1e-6)
            prev_time = time.time()
            cv2.putText(frame, f"FPS: {fps:.1f}  STATE: {state}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            cv2.imshow("Robot View", frame)
            cv2.imshow("Blue Mask", mask)
            cv2.imshow("Red Mask", mask_red)

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
