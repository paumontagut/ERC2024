#!/bin/bash
# manage_services.sh
# =====================================================
# Script interactivo para gestionar servicios del rover.
# Se definen los servicios en un array para facilitar futuras modificaciones.
# =====================================================

if [[ $EUID -ne 0 ]]; then
    echo "❌ Este script debe ejecutarse con sudo o como root."
    exit 1
fi

# Directorio donde se encuentran los archivos de servicio en el repositorio
SYSTEMD_DIR="/home/ujiroboticsteam/ERC2024/systemd"
TARGET_DIR="/etc/systemd/system"

# Lista de servicios (sin extensión, si hay timers se manejarán por separado)
SERVICES=(
  "rover-devices-setup.service"
  "rover-file-receptor.service"
  "rover-internet.service"
  "rover-ip.service"          # El servicio que se usará con timer
  # "rover-sshx.service"
  "rover-supervisor.service"
)

# Si usas el timer para ip, también lo puedes agregar a una variable aparte:
TIMERS=(
  "rover-ip.timer"
)

# Función para instalar (copiar) los archivos de servicio y timer
install_services() {
  echo "Copiando archivos de servicio..."
  for file in "$SYSTEMD_DIR"/*.service; do
    sudo cp "$file" "$TARGET_DIR"
    echo "Instalado: $(basename "$file")"
  done
  for file in "$SYSTEMD_DIR"/*.timer; do
    sudo cp "$file" "$TARGET_DIR"
    echo "Instalado: $(basename "$file")"
  done
  echo "Recargando daemon de systemd..."
  sudo systemctl daemon-reload
}

# Función para habilitar todos los servicios y timers
enable_services() {
  for service in "${SERVICES[@]}"; do
    sudo systemctl enable "$service"
  done
  for timer in "${TIMERS[@]}"; do
    sudo systemctl enable "$timer"
  done
}

# Función para iniciar todos los servicios y timers
start_services() {
  for service in "${SERVICES[@]}"; do
    sudo systemctl start "$service"
  done
  for timer in "${TIMERS[@]}"; do
    sudo systemctl start "$timer"
  done
}

# Función para detener todos los servicios y timers
stop_services() {
  for service in "${SERVICES[@]}"; do
    sudo systemctl stop "$service"
  done
  for timer in "${TIMERS[@]}"; do
    sudo systemctl stop "$timer"
  done
}

# Función para reiniciar todos los servicios y timers
restart_services() {
  stop_services
  start_services
}

# Función para mostrar el estado de todos los servicios y timers
status_services() {
  for service in "${SERVICES[@]}"; do
    sudo systemctl status "$service" --no-pager
    echo "-------------------------------------------"
  done
  for timer in "${TIMERS[@]}"; do
    sudo systemctl status "$timer" --no-pager
    echo "-------------------------------------------"
  done
}

# Menú interactivo
while true; do
  echo -e "\nGestión de Servicios del Rover"
  echo "1) Instalar servicios (copiar archivos y reload)"
  echo "2) Habilitar servicios (arranque automático)"
  echo "3) Iniciar servicios"
  echo "4) Detener servicios"
  echo "5) Reiniciar servicios"
  echo "6) Mostrar estado de servicios"
  echo "7) Salir"
  read -rp "Selecciona una opción [1-7]: " option

  case $option in
    1)
      install_services
      ;;
    2)
      enable_services
      ;;
    3)
      start_services
      ;;
    4)
      stop_services
      ;;
    5)
      restart_services
      ;;
    6)
      status_services
      ;;
    7)
      echo "Saliendo..."
      exit 0
      ;;
    *)
      echo "Opción no válida. Prueba otra vez."
      ;;
  esac
done
