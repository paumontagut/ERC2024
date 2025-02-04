#!/bin/bash

# Archivo temporal para almacenar los IDs de los mensajes enviados
PERSISTENT_MSG_LOG="/var/lib/jetson_bot/last_message_id.log"
MESSAGES_LOG="/tmp/telegram_message_ids.log"

# Ensure directory exists
mkdir -p /var/lib/jetson_bot

# Función para borrar todos los mensajes registrados en MESSAGES_LOG
delete_all_previous_messages() {
    if [[ -f "$MESSAGES_LOG" ]]; then
        while read -r msg_id; do
            if [[ -n "$msg_id" ]]; then
                curl -s -o /dev/null -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/deleteMessage" \
                     -d chat_id="$TELEGRAM_CHAT_ID" \
                     -d message_id="$msg_id"
                echo "🗑️ Mensaje con ID $msg_id eliminado."
            fi
        done < "$MESSAGES_LOG"
        rm -f "$MESSAGES_LOG"
    fi
}

store_message_id() {
    echo "$1" >> "$MESSAGES_LOG"
}

store_shutdown_message_id() {
    echo "$1" > "$PERSISTENT_MSG_LOG"
}

delete_shutdown_message() {
    if [[ -f "$PERSISTENT_MSG_LOG" ]]; then
        LAST_MSG_ID=$(cat "$PERSISTENT_MSG_LOG")
        if [[ -n "$LAST_MSG_ID" ]]; then
            curl -s -o /dev/null -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/deleteMessage" \
                 -d chat_id="$TELEGRAM_CHAT_ID" \
                 -d message_id="$LAST_MSG_ID"
            echo "🗑️ Deleted previous shutdown message ID: $LAST_MSG_ID"
            rm -f "$PERSISTENT_MSG_LOG"
        fi
    fi
}


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

    # Borrar todos los mensajes anteriores
    delete_all_previous_messages

    # Enviar un nuevo mensaje indicando que la Jetson está inactiva
    MESSAGE_DOWN="La Jetson está actualmente tomándose una siesta 💤💤💤💤"
    RESPONSE=$(curl -s -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/sendMessage" \
         -d chat_id="$TELEGRAM_CHAT_ID" \
         -d text="$MESSAGE_DOWN")
    SHUTDOWN_MSG_ID=$(echo "$RESPONSE" | jq -r '.result.message_id')
    echo "📩 Mensaje de inactividad enviado con ID: $NEW_MSG_ID"
    store_shutdown_message_id "$SHUTDOWN_MSG_ID"
    
    if [[ -n "$SSHX_PID" ]]; then
        kill "$SSHX_PID"
    fi
    rm -f "$TEMP_OUTPUT"
    exit 0
}

# CTRL + C (SIGINT) or termination signal -> and call cleanup()
trap cleanup SIGINT SIGTERM SIGABRT

# Delete the last shutdown message before sending new SSHX link
delete_shutdown_message

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

    # Borrar todos los mensajes anteriores antes de enviar el nuevo
    delete_all_previous_messages

    # Enviar el nuevo mensaje a Telegram
    RESPONSE=$(curl -s -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/sendMessage" \
         -d chat_id="$TELEGRAM_CHAT_ID" \
         -d text="$MESSAGE")
    NEW_MSG_ID=$(echo "$RESPONSE" | jq -r '.result.message_id')
    echo "📩 Mensaje enviado con ID: $NEW_MSG_ID"
    store_message_id "$NEW_MSG_ID"
else
    echo "❌ Failed to extract the SSHX link."
fi

# Keep sshx running indefinitely
echo "🟢 sshx is running... Press CTRL + C to stop."
wait "$SSHX_PID"
