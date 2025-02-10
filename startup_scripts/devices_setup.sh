#!/bin/bash
set -e

# Ensure the script is executed as root.
if [[ $EUID -ne 0 ]]; then
    echo "❌ Este script debe ejecutarse con sudo o como root."
    exit 1
fi

# Define the location of the udev rules file.
UDEV_RULES_FILE="/etc/udev/rules.d/99-usb.rules"

echo "🔄 Creando reglas UDEV en $UDEV_RULES_FILE ..."

cat <<EOF > "$UDEV_RULES_FILE"
# LIDAR 4D
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ENV{ID_MODEL}=="CP2102N_USB_to_UART_Bridge_Controller", SYMLINK+="lidar2d", MODE="0666"

# LIDAR 2D
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ENV{ID_MODEL}=="CP2104_USB_to_UART_Bridge_Controller", SYMLINK+="unitree_lidar", MODE="0666"

# Ruedas (Motor)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT94VXEO", SYMLINK+="ruedas", MODE="0666"

# Webcam Logitech C270 Derecha (capture node)
SUBSYSTEM=="video4linux", \
    ENV{ID_V4L_CAPABILITIES}==":capture:", \
    ENV{ID_PATH}=="platform-3610000.usb-usb-0:2.3.3:1.0", \
    SYMLINK+="logitech_derecha", MODE="0666"

# Webcam Logitech C270 Izquierda (capture node)
SUBSYSTEM=="video4linux", \
    ENV{ID_V4L_CAPABILITIES}==":capture:", \
    ENV{ID_PATH}=="platform-3610000.usb-usb-0:2.3.4:1.0", \
    SYMLINK+="logitech_izquierda", MODE="0666"
EOF

echo "🔄 Recargando reglas de udev..."
udevadm control --reload-rules
udevadm trigger

echo "✅ Reglas UDEV aplicadas. Verificando..."
ls -l /dev/lidar2d /dev/unitree_lidar /dev/ruedas /dev/realsense_depth /dev/realsense_color /dev/logitech_izquierda /dev/logitech_derecha 2>/dev/null

echo "🟢 Configuración completada."
