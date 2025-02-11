#!/bin/bash
set -e

# Asegurarse de que se ejecuta como root.
if [[ $EUID -ne 0 ]]; then
    echo "❌ Este script debe ejecutarse con sudo o como root."
    exit 1
fi

# Verificar que sshx esté instalado.
if ! command -v sshx &> /dev/null; then
    echo "sshx not found, installing..."
    curl -sSf https://sshx.io/get | sh
fi

# Inicializar la variable MESSAGE_ID.
MESSAGE_ID=""

# Configuración del Bot de Telegram (actualizar con tus datos).
TELEGRAM_BOT_TOKEN="7578036836:AAFAlp3rRw8pRo6-veardgQN2MZRoBcvlyA"  # Reemplazar con el token real.
TELEGRAM_CHAT_ID="-1002293462923"  # Reemplazar con el chat o grupo.

# Función cleanup: ahora simplemente mata sshx y limpia el archivo temporal.
cleanup() {
    echo "🔴 Stopping sshx..."
    if [[ -n "$SSHX_PID" ]]; then
        kill "$SSHX_PID"
    fi
    rm -f "$TEMP_OUTPUT"
    exit 1
}

# Atrapar SIGTERM y SIGABRT para limpiar.
trap cleanup SIGTERM SIGABRT

# Iniciar sshx en segundo plano y capturar su salida en un archivo temporal.
TEMP_OUTPUT=$(mktemp)
sshx > "$TEMP_OUTPUT" 2>&1 &
SSHX_PID=$!

# Esperar hasta que se detecte el enlace generado por sshx.
TIMEOUT=30    # Tiempo máximo en segundos.
WAITED=0
echo "Waiting for sshx link to be generated..."
while true; do
    LINK=$(grep -oP 'https://sshx\.io/s/\S+' "$TEMP_OUTPUT" | sed 's/\x1b\[[0-9;]*m//g' | tr -d '[:cntrl:]')
    if [[ -n "$LINK" ]]; then
        break
    fi
    sleep 1
    WAITED=$((WAITED+1))
    if [ "$WAITED" -ge "$TIMEOUT" ]; then
        echo "Timeout waiting for sshx link"
        exit 1
    fi
done

echo "✅ SSHX Link detected: $LINK"

# Obtener estadísticas del sistema.
WIFI_SSID=$(iwgetid -r 2>/dev/null)
if [[ -z "$WIFI_SSID" ]]; then
    WIFI_SSID="(No se detecta conexión WiFi)"
fi

LOCAL_IP=$(hostname -I | awk '{print $1}')
if [[ -z "$LOCAL_IP" ]]; then
    LOCAL_IP="(No se detecta IP)"
fi

# Preparar el mensaje para Telegram.
MESSAGE="🚀 ¡El rover está listo para ser controlado! 🌍

🔗 Accede a su terminal de forma remota con este enlace:
👉 $LINK

📡 *Red WiFi:* \`$WIFI_SSID\`
🌐 *IP Local:* \`$LOCAL_IP\`

📌 Este enlace permanecerá activo mientras la Jetson siga encendida y no se cancele el sshx.

🤖 ¡A programar!"

# Enviar el mensaje a Telegram (sin borrar mensajes previos).
RESPONSE=$(curl -s -X POST "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/sendMessage" \
         -d chat_id="${TELEGRAM_CHAT_ID}" \
         -d text="${MESSAGE}")
MESSAGE_ID=$(echo "$RESPONSE" | jq -r '.result.message_id')
echo "📩 Mensaje enviado con ID: ${MESSAGE_ID}"

echo "🟢 sshx is running... The process will remain active."
# Mantener el proceso activo sin generar nuevos enlaces.
tail -f /dev/null
