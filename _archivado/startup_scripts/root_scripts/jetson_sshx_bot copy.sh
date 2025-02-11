#!/bin/bash

# Ensure the script is executed as root.
if [[ $EUID -ne 0 ]]; then
    echo "❌ Este script debe ejecutarse con sudo o como root."
    exit 1
fi

# Ensure sshx is installed
if ! command -v sshx &> /dev/null; then
    echo "sshx not found, installing..."
    curl -sSf https://sshx.io/get | sh
fi

# Initialize MESSAGE_ID variable
MESSAGE_ID=""

# Telegram Bot Configuration
TELEGRAM_BOT_TOKEN="7578036836:AAFAlp3rRw8pRo6-veardgQN2MZRoBcvlyA"  # Reemplazar con el token del bot
TELEGRAM_CHAT_ID="-1002293462923"  # Reemplazar con el chat ID o grupo

cleanup() {
    echo "🔴 Stopping sshx..."

    # Delete the message with the link
    if [[ -n "$MESSAGE_ID" ]]; then
        curl -s -o /dev/null -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/deleteMessage" \
             -d chat_id="$TELEGRAM_CHAT_ID" \
             -d message_id="$MESSAGE_ID"
        echo "🗑️ Mensaje del enlace eliminado."
    fi

    # Send a new message saying the Jetson is not active
    MESSAGE_DOWN="La Jetson está actualmente tomándose una siesta 💤💤💤💤"
    RESPONSE=$(curl -s -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/sendMessage" \
         -d chat_id="$TELEGRAM_CHAT_ID" \
         -d text="$MESSAGE_DOWN")

    MESSAGE_ID=$(echo "$RESPONSE" | jq -r '.result.message_id')
    echo "📩 Mensaje de inactividad enviado con ID: $MESSAGE_ID"

    if [[ -n "$SSHX_PID" ]]; then
        kill "$SSHX_PID"
    fi
    rm -f "$TEMP_OUTPUT"
    exit 1
}

# CTRL + C (SIGINT) or termination signal -> and call cleanup()
trap cleanup SIGINT SIGTERM SIGABRT

# Start sshx in the background
TEMP_OUTPUT=$(mktemp)
sshx > "$TEMP_OUTPUT" 2>&1 &
SSHX_PID=$!




# Wait until the output contains the generated link.
TIMEOUT=30    # Timeout en segundos.
WAITED=0
echo "Waiting for sshx link to be generated..."
while true; do
    # Intenta extraer la línea que contiene el enlace.
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


# -----------------------------
# Obtener estadísticas del sistema
# -----------------------------

# 1️⃣ WiFi SSID
WIFI_SSID=$(iwgetid -r 2>/dev/null)
if [[ -z "$WIFI_SSID" ]]; then
    WIFI_SSID="(No se detecta conexión WiFi)"
fi

# 2️⃣ Dirección IP local
LOCAL_IP=$(hostname -I | awk '{print $1}')
if [[ -z "$LOCAL_IP" ]]; then
    LOCAL_IP="(No se detecta IP)"
fi

# -----------------------------
# Enviar mensaje a Telegram
# -----------------------------

# Send the improved SSHX link message to Telegram and store the message ID

MESSAGE="🚀 ¡El rover está listo para ser controlado! 🌍

🔗 Accede a su terminal de forma remota con este enlace:
👉 $LINK

📡 *Red WiFi:* \`$WIFI_SSID\`
🌐 *IP Local:* \`$LOCAL_IP\`

📌 Este enlace permanecerá activo mientras la Jetson siga encendida y no se cancele el sshx.

🤖 ¡A programar!"

# Si existiera algún mensaje anterior (en principio MESSAGE_ID está vacío), se elimina.
if [[ -n "$MESSAGE_ID" ]]; then
    curl -s -o /dev/null -X POST "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/deleteMessage" \
         -d chat_id="${TELEGRAM_CHAT_ID}" \
         -d message_id="${MESSAGE_ID}"
    echo "🗑️ Mensaje previo eliminado."
fi

RESPONSE=$(curl -s -X POST "https://api.telegram.org/bot${TELEGRAM_BOT_TOKEN}/sendMessage" \
         -d chat_id="${TELEGRAM_CHAT_ID}" \
         -d text="${MESSAGE}")
MESSAGE_ID=$(echo "$RESPONSE" | jq -r '.result.message_id')
echo "📩 Mensaje enviado con ID: ${MESSAGE_ID}"

echo "🟢 sshx is running... The process will remain active."
while true; do sleep 3600; done