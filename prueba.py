import cv2

VIDEO_URL = "http://192.168.4.1:81/stream" 

cap = cv2.VideoCapture(VIDEO_URL)
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

print("isOpened:", cap.isOpened())

ret, frame = cap.read()
print("ret:", ret)

if ret and frame is not None:
    print("frame shape:", frame.shape)
else:
    print("No frame received.")

cap.release()
