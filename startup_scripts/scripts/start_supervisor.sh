#!/bin/bash

# Redirect all output (stdout and stderr) to a log file
LOG_FILE="/home/ujiroboticsteam/ERC2024/utilidades/start_supervisor.log"
exec > >(tee -a "$LOG_FILE") 2>&1
echo "Starting Supervisor Script at $(date)"

# Store the sudo password in a variable (use cautiously)
SUDO_PASS="qwerty"

# --- 1. Verificar y asegurar conexión a Internet ---
# Activar NTP para sincronización de hora
echo "$SUDO_PASS" | sudo -S timedatectl set-ntp false
echo "$SUDO_PASS" | sudo -S timedatectl set-ntp true

# Verificar si está conectado a Internet (probando con Google DNS)
echo "Verificando conexión a Internet..."
if ping -q -c 1 -W 1 8.8.8.8 >/dev/null; then
    echo "Conexión a Internet exitosa."
else
    echo "Conexión a Internet fallida. Intentando reconectar..."
    # Conectar a Wi-Fi
    if nmcli dev status | grep -q wifi; then
        echo "Reconectando a la red Wi-Fi..."
        nmcli dev wifi connect "ego" password "avoveavo"
        sleep 5
    fi
    # Comprobar nuevamente la conexión
    if ping -q -c 1 -W 1 8.8.8.8 >/dev/null; then
        echo "Reconexión exitosa."
    else
        echo "No se pudo establecer conexión a Internet. Continuando sin conexión."
    fi
fi

# --- 2. Configurar entorno ROS2 ---
source /opt/ros/humble/setup.bash
source /home/ujiroboticsteam/ERC2024/erc_ws/install/setup.bash

# --- 3. Configurar ROS_DOMAIN_ID ---
#export ROS_DOMAIN_ID=0

# --- 4. Lanzar nodo supervisor ---
exec ros2 run rover_supervisor supervisor_node

echo -e "Supervisor Script finished at $(date)\n"