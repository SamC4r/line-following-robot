import socket
import json

ip = "192.168.4.1"
port = 100

car = socket.socket()
car.connect((ip, port))
print("Conectado!")

# Mover hacia adelante
cmd = {"H": 1, "N": 4, "D1": 150, "D2": 150}
car.send((json.dumps(cmd) + "\n").encode())
print("Comando enviado")