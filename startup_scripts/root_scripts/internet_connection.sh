#!/bin/bash
# =============================================================================
# Script de monitoreo y reconexión de Internet
#
# Este script verifica constantemente la conexión a Internet y, en caso de
# fallo, intenta reconectar primero a la red principal (WifiRover). Si tras
# varios intentos (umbral configurable) la conexión no se recupera, se conecta
# a la red de respaldo (rover). Se ejecuta en segundo plano consumiendo
# recursos mínimos.
#
# Nota: Si existen varios puntos de acceso con el mismo SSID, nmcli se
#       conectará al que esté configurado o encuentre primero, por lo que se
#       recomienda ajustar las prioridades en NetworkManager si fuera necesario.
# =============================================================================

# -------------------- Configuración --------------------
# Redes (modifica las variables según tu entorno)
PRIMARY_SSID="AntenaRover"
PRIMARY_PASS="wifirover777"

BACKUP_SSID="ego"
BACKUP_PASS="avoveavo"   # Cambia según la contraseña que se use en móviles

# Umbral de fallos consecutivos antes de intentar la red de respaldo
FAILURE_THRESHOLD=3

# Intervalo (en segundos) entre comprobaciones
CHECK_INTERVAL=3

# -------------------- Funciones --------------------
# Comprueba la conexión a Internet realizando un ping a 8.8.8.8
check_internet_connection() {
    if ping -q -c 1 -W 1 8.8.8.8 >/dev/null 2>&1; then
        return 0
    else
        return 1
    fi
}

# Obtiene el SSID de la red WiFi a la que se está conectado actualmente
get_current_ssid() {
    # nmcli devuelve líneas como "yes:WifiRover". Se extrae el SSID.
    nmcli -t -f active,ssid dev wifi | awk -F: '$1=="yes" {print $2; exit}'
}

# Intenta conectarse a una red dada usando nmcli
connect_to_network() {
    local ssid="$1"
    local password="$2"
    echo "$(date): Attempting to connect to '$ssid'..."
    nmcli dev wifi connect "$ssid" password "$password"
    if [ $? -eq 0 ]; then
        echo "$(date): Successfully connected to '$ssid'."
    else
        echo "$(date): Error connecting to '$ssid'."
    fi
    sleep 5
}


# Función principal que monitorea la conexión y gestiona las reconexiones
monitor_connection() {
    local failure_count=0

    while true; do
        if check_internet_connection; then
            # Si hay conexión, se reinicia el contador
            if [ $failure_count -ne 0 ]; then
                echo "$(date): Conexión a Internet recuperada."
            fi
            failure_count=0
        else
            # Si no hay conexión, se incrementa el contador y se notifica
            failure_count=$((failure_count+1))
            echo "$(date): Fallo en conexión a Internet. Intento $failure_count de $FAILURE_THRESHOLD."

            # Si no se está conectado a la red principal, se intenta reconectar
            current_ssid=$(get_current_ssid)
            if [ "$current_ssid" != "$PRIMARY_SSID" ]; then
                connect_to_network "$PRIMARY_SSID" "$PRIMARY_PASS"
            fi

            # Si se supera el umbral, se intenta cambiar a la red de respaldo
            if [ "$failure_count" -ge "$FAILURE_THRESHOLD" ]; then
                current_ssid=$(get_current_ssid)
                if [ "$current_ssid" != "$BACKUP_SSID" ]; then
                    echo "$(date): Umbral superado. Conectando a red de respaldo '$BACKUP_SSID'."
                    connect_to_network "$BACKUP_SSID" "$BACKUP_PASS"
                fi
            fi
        fi
        sleep "$CHECK_INTERVAL"
    done
}

# -------------------- Manejador de Señales --------------------
# Permite que el script termine de forma limpia al recibir SIGINT o SIGTERM.
trap 'echo "$(date): Terminando script de monitoreo."; exit 0' SIGINT SIGTERM

# -------------------- Inicio del Script --------------------
echo "$(date): Iniciando monitoreo de conexión a Internet."
monitor_connection
