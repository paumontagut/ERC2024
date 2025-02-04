import socket
import os

# Configuración del servidor
HOST = "192.168.131.228"  # IP del receptor
PORT = 5001               # Puerto para escuchar

# Crear el socket del servidor
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
print(f"Servidor escuchando en {HOST}:{PORT}")

while True:
    print("Esperando conexión...")
    conn, addr = server_socket.accept()
    print(f"Conexión recibida de: {addr}")

    # Recibir el nombre del archivo
    file_name = conn.recv(1024).decode()
    print(f"Recibiendo archivo: {file_name}")

    # Abrir el archivo para escribir los datos recibidos
    with open(file_name, "wb") as file:
        while True:
            data = conn.recv(4096)  # Tamaño del bloque
            if not data:
                break
            file.write(data)

    print(f"Archivo {file_name} recibido con éxito.")
    conn.close()
