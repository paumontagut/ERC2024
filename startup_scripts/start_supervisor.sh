#!/bin/bash

set -e

echo "Starting Supervisor Script at $(date)"

# --- 2. Configurar entorno ROS2 ---
source /opt/ros/humble/setup.bash
source /home/ujiroboticsteam/ERC2024/erc_ws/install/setup.bash

# --- 3. Configurar ROS_DOMAIN_ID ---
unset ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0

# --- 4. Lanzar nodo supervisor ---
exec ros2 run rover_supervisor supervisor_node
