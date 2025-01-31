#!/bin/bash

# Ensure sshx is installed
if ! command -v sshx &> /dev/null; then
    echo "sshx not found, installing..."
    curl -sSf https://sshx.io/get | sh
fi

# Start sshx in the background
TEMP_OUTPUT=$(mktemp)
sshx > "$TEMP_OUTPUT" 2>&1 &

# Wait 5 seconds to allow sshx to generate the link
sleep 5

# Extract the SSHX link from the output
LINK=$(grep -oP 'https://sshx\.io/s/\S+' "$TEMP_OUTPUT")

# Telegram Bot Configuration
TELEGRAM_BOT_TOKEN="7578036836:AAFAlp3rRw8pRo6-veardgQN2MZRoBcvlyA"  # Replace with your actual bot token
TELEGRAM_CHAT_ID="-4700889559"  # Use your personal chat ID OR group ID (-987654321 for groups)

# Send the SSHX link to Telegram
if [[ -n "$LINK" ]]; then
    echo "✅ SSHX Link: $LINK"

    curl -s -X POST "https://api.telegram.org/bot$TELEGRAM_BOT_TOKEN/sendMessage" \
         -d chat_id="$TELEGRAM_CHAT_ID" \
         -d text="Jetson SSHX Link: $LINK"

else
    echo "❌ Failed to extract the SSHX link."
fi

# Keep sshx running (DO NOT terminate it)
rm -f "$TEMP_OUTPUT"
