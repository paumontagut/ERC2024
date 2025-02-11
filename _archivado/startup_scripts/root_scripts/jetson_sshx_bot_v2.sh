#!/bin/bash

# Ensure sshx is installed
if ! command -v sshx &> /dev/null; then
    echo "sshx not found, installing..."
    curl -sSf https://sshx.io/get | sh
fi

# Function to clean up sshx when script is stopped
cleanup() {
    echo "🔴 Stopping sshx..."
    if [[ -n "$SSHX_PID" ]]; then
        kill "$SSHX_PID"
    fi
    rm -f "$TEMP_OUTPUT"
    exit 0
}

# Trap CTRL + C (SIGINT) and call cleanup()
trap cleanup SIGINT

# Start sshx in the background
TEMP_OUTPUT=$(mktemp)
sshx > "$TEMP_OUTPUT" 2>&1 &
SSHX_PID=$!

# Wait 5 seconds to allow sshx to generate the link
sleep 5

# Extract the SSHX link from the output
LINK=$(grep -oP 'https://sshx\.io/s/\S+' "$TEMP_OUTPUT" | sed 's/\x1b\[[0-9;]*m//g' | tr -d '[:cntrl:]')

# Telegram Bot Configuration
TELEGRAM_BOT_TOKEN="7578036836:AAFAlp3rRw8pRo6-veardgQN2MZRoBcvlyA"  # Replace with your actual bot token
TELEGRAM_CHAT_ID="-4700889559"  # Use your personal chat ID OR group ID (-987654321 for groups)

# Send the SSHX link to Telegram
if [[ -n "$LINK" ]]; then
    echo "✅ SSHX Link: $LINK"

    MESSAGE="🚀 ¡El rover ya está listo para ser controlado! 🌍

    🔗 Accede a su terminal de forma remota con este enlace:
    👉 $LINK

    📌 Este enlace permanecerá activo mientras la Jetson siga encendida y no se cancele el sshx.

    🤖 Ahora... ¡A programar!"

    curl -s -o /dev/null -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/sendMessage" \
         -d chat_id="$TELEGRAM_CHAT_ID" \
         -d text="$MESSAGE"

else
    echo "❌ Failed to extract the SSHX link."
fi

# Keep sshx running indefinitely
echo "🟢 sshx is running... Press CTRL + C to stop."
wait "$SSHX_PID"
