#!/bin/bash

# Verificar si el script se está ejecutando como root
if [[ $EUID -ne 0 ]]; then
    echo "❌ Este script debe ejecutarse con sudo o como root."
    exit 1
fi

# Definir la ubicación del archivo de reglas
UDEV_RULES_FILE="/etc/udev/rules.d/99-usb.rules"

# Escribir las reglas en el archivo
echo "🔄 Creando reglas UDEV en $UDEV_RULES_FILE ..."

cat <<EOF > $UDEV_RULES_FILE
# 1. LIDARs
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="02C90019", SYMLINK+="unitree_lidar", MODE="0666"

# 2. Ruedas (Motor)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT94VXEO", SYMLINK+="ruedas", MODE="0666"

# 3. Cámaras RealSense Depth
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", ATTRS{serial}=="243423020287", SYMLINK+="realsense_depth", MODE="0666"

# 4. Cámaras RealSense Color (si tienen seriales únicos)
SUBSYSTEM=="video4linux", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", ATTRS{serial}=="243423020287", SYMLINK+="realsense_color", MODE="0666"

# 5. Webcam Logitech C270 Izquierda
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="platform-3610000.usb-usb-0:2.3.3:1.0", SYMLINK+="logitech_izquierda", MODE="0666"

# 6. Webcam Logitech C270 Derecha
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="platform-3610000.usb-usb-0:2.3.4:1.0", SYMLINK+="logitech_derecha", MODE="0666"
EOF

# Aplicar cambios en udev
echo "🔄 Recargando reglas de udev..."
udevadm control --reload-rules
udevadm trigger

# Mostrar reglas creadas
echo "✅ Reglas UDEV aplicadas. Verificando..."
ls -l /dev/unitree_lidar /dev/ruedas /dev/realsense_depth /dev/realsense_color /dev/logitech_izquierda /dev/logitech_derecha 2>/dev/null

echo "🟢 Configuración completada."
