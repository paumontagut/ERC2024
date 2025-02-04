#!/bin/bash

# Verificar si el script se está ejecutando como root
if [[ $EUID -ne 0 ]]; then
    echo "❌ Este script debe ejecutarse con sudo o como root."
    exit 1
fi

# Definir la ubicación del archivo de reglas
rm -f /etc/udev/rules.d/99-usb.rules

# Aplicar cambios en udev
echo "🔄 Recargando reglas de udev..."
udevadm control --reload-rules
udevadm trigger

# Mostrar reglas creadas
echo "✅ Reglas UDEV aplicadas. Verificando..."
ls -l /dev/unitree_lidar /dev/ruedas /dev/realsense_depth /dev/realsense_color /dev/logitech_izquierda /dev/logitech_derecha 2>/dev/null

echo "🟢 Configuración completada."
