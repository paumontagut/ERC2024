import socket
import os
import subprocess

def get_downloads_folder():
    """Obtiene la ruta de la carpeta Downloads independientemente del idioma del sistema."""
    try:
        result = subprocess.run(["xdg-user-dir", "DOWNLOAD"], stdout=subprocess.PIPE, text=True)
        downloads_dir = result.stdout.strip()
        if os.path.exists(downloads_dir):
            return downloads_dir
    except FileNotFoundError:
        print("El comando 'xdg-user-dir' no está disponible. Usando ruta predeterminada.")

    home_dir = os.path.expanduser("~")
    return os.path.join(home_dir, "Downloads")

def ensure_received_files_directory():
    """Crea la carpeta 'Received Files' dentro de la carpeta de Descargas si no existe."""
    downloads_dir = get_downloads_folder()
    received_files_dir = os.path.join(downloads_dir, "Received Files")

    if not os.path.exists(received_files_dir):
        os.makedirs(received_files_dir)
        print(f"Carpeta creada: {received_files_dir}")
    else:
        print(f"Carpeta existente: {received_files_dir}")

    return received_files_dir

def get_local_ip():
    """Obtiene la dirección IP local del dispositivo."""
    try:
        temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        temp_socket.connect(("8.8.8.8", 80))
        local_ip = temp_socket.getsockname()[0]
        temp_socket.close()
        return local_ip
    except Exception as e:
        print(f"Error al obtener la IP local: {e}")
        return None

# Configuración del servidor
HOST = get_local_ip()
if not HOST:
    print("No se pudo obtener la dirección IP local. Cerrando el servidor.")
    exit(1)
PORT = 5001

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)
print(f"Servidor escuchando en {HOST}:{PORT}")

received_files_directory = ensure_received_files_directory()

while True:
    print("Esperando conexión...")
    conn, addr = server_socket.accept()
    print(f"Conexión recibida de: {addr}")

    # Recibir el nombre del archivo correctamente
    file_name = ""
    while not file_name.endswith("\n"):
        file_name += conn.recv(1).decode()

    file_name = file_name.strip()  # Eliminar caracteres de nueva línea y espacios extra
    print(f"Recibiendo archivo: {file_name}")

    file_path = os.path.join(received_files_directory, file_name)

    # Abrir archivo para escribir
    with open(file_path, "wb") as file:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            file.write(data)

    print(f"Archivo {file_name} recibido con éxito en {file_path}")
    conn.close()
