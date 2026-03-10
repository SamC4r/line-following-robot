import cv2

VIDEO_URL = "http://192.168.2.242:8080/video"   # change this

cap = cv2.VideoCapture(VIDEO_URL)

print("isOpened:", cap.isOpened())

ret, frame = cap.read()
print("ret:", ret)

if ret and frame is not None:
    print("frame shape:", frame.shape)
else:
    print("No frame received.")

cap.release()
