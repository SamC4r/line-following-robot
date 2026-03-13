import serial #pyserial
import time


SERIAL_PORT = "COM3"#"/dev/ttyUSB0"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)


cmd = [
    "M,160,160",
    "M,-160,-160",
    "M,0,0",
]

for c in cmd:
    print("Sending" , c)
    ser.write((c + "\n").encode())
    time.sleep(0.2)
