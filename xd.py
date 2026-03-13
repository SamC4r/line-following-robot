import socket

ports_to_try = [  80]

for port in ports_to_try:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(5)
        print(f"[OK] Escuchando en puerto {port}...")
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                print(f"Puerto {port} - Desde {addr}: {data}")
            except socket.timeout:
                pass
    except OSError:
        print(f"[SKIP] Puerto {port} ocupado")
        sock.close()