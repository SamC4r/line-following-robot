import socket
import threading
import time

# ── Connect ───────────────────────────────────────────────────
car = socket.socket()
car.connect(("192.168.4.1", 100))
print("Connected!")

# ── Heartbeat (background thread) ─────────────────────────────
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
                    print("♥")
                else:
                    print("Robot:", msg)
        except:
            break

threading.Thread(target=receiver, daemon=True).start()

# ── Command helper ─────────────────────────────────────────────
def motor(left, right):
    cmd = "{" + f"M,{left},{right}" + "}"
    car.send(cmd.encode())
    print("Sent:", cmd)

def stop():
    motor(0, 0)

motor(200,200)
car.close()