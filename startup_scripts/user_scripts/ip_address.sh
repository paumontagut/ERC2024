#!/bin/bash

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0
unset ROS_DOMAIN_ID

# Bucle infinito para enviar la IP cada 10 segundos
while true; do
    # Obtener la dirección IP local
    IP_ADDRESS=$(hostname -I | awk '{print $1}')

    # Si no se pudo obtener la IP, establecer un valor por defecto
    if [[ -z "$IP_ADDRESS" ]]; then
        IP_ADDRESS="(No se detecta IP)"
    fi

    # Publicar la IP en el topic de ROS 2
    ros2 topic pub /ip_variable std_msgs/msg/String "{data: '$IP_ADDRESS'}" --once > /dev/null 2>&1

    # Mostrar en la terminal la IP enviada
    echo "📡 Dirección IP enviada: $IP_ADDRESS"

    # Esperar 10 segundos antes de enviar nuevamente
    sleep 10
done
