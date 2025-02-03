#!/bin/bash

# Ensure sshx is installed
if ! command -v sshx &> /dev/null; then
    echo "sshx not found, installing..."
    curl -sSf https://sshx.io/get | sh
fi

# Telegram Bot Configuration
TELEGRAM_BOT_TOKEN="7578036836:AAFAlp3rRw8pRo6-veardgQN2MZRoBcvlyA"  # Reemplazar con el token del bot
TELEGRAM_CHAT_ID="-1002293462923"  # Reemplazar con el chat ID o grupo

# Function to clean up sshx when script is stopped
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
    exit 0
}

# CTRL + C (SIGINT) or termination signal -> and call cleanup()
trap cleanup SIGINT SIGTERM SIGABRT

# Start sshx in the background
TEMP_OUTPUT=$(mktemp)
sshx > "$TEMP_OUTPUT" 2>&1 &
SSHX_PID=$!

# Wait 5 seconds to allow sshx to generate the link
sleep 5

# Extract the SSHX link and ensure no hidden characters appear
LINK=$(grep -oP 'https://sshx\.io/s/\S+' "$TEMP_OUTPUT" | sed 's/\x1b\[[0-9;]*m//g' | tr -d '[:cntrl:]')

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
if [[ -n "$LINK" ]]; then
    echo "✅ SSHX Link: $LINK"

    MESSAGE="🚀 ¡El rover está listo para ser controlado! 🌍

🔗 Accede a su terminal de forma remota con este enlace:
👉 $LINK

📡 *Red WiFi:* \`$WIFI_SSID\`
🌐 *IP Local:* \`$LOCAL_IP\`

📌 Este enlace permanecerá activo mientras la Jetson siga encendida y no se cancele el sshx.

🤖 ¡A programar!"

    # Borrar mensaje anterior si existe
    if [[ -n "$MESSAGE_ID" ]]; then
        curl -s -o /dev/null -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/deleteMessage" \
             -d chat_id="$TELEGRAM_CHAT_ID" \
             -d message_id="$MESSAGE_ID"
        echo "🗑️ Mensaje previo eliminado."
    fi

    # Enviar nuevo mensaje a Telegram
    RESPONSE=$(curl -s -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/sendMessage" \
         -d chat_id="$TELEGRAM_CHAT_ID" \
         -d text="$MESSAGE")

    MESSAGE_ID=$(echo "$RESPONSE" | jq -r '.result.message_id')
    echo "📩 Mensaje enviado con ID: $MESSAGE_ID"

else
    echo "❌ Failed to extract the SSHX link."
fi

# Keep sshx running indefinitely
echo "🟢 sshx is running... Press CTRL + C to stop."
wait "$SSHX_PID"
