#!/bin/bash

ROOT_SCRIPTS_DIR="/home/ujiroboticsteam/ERC2024/startup_scripts/root_scripts"
USER_SCRIPTS_DIR="/home/ujiroboticsteam/ERC2024/startup_scripts/user_scripts"

LOGFILE="/home/ujiroboticsteam/ERC2024/startup_scripts/startup_scripts.log"
IGNORE_FILE="/home/ujiroboticsteam/ERC2024/startup_scripts/ignored_scripts.txt"

echo "🔄 Starting startup scripts at $(date)" >> "$LOGFILE"

# Read ignored scripts (if any)
if [[ -f "$IGNORE_FILE" ]]; then
    mapfile -t IGNORED_SCRIPTS < "$IGNORE_FILE"
else
    IGNORED_SCRIPTS=()
fi

run_scripts() {
    local SCRIPTS_DIR="$1"
    local RUN_AS_ROOT="$2"

    for script in "$SCRIPTS_DIR"/*.sh; do
        script_name=$(basename "$script")

        # Skip ignored scripts
        if [[ " ${IGNORED_SCRIPTS[@]} " =~ " $script_name " ]]; then
            echo "Skipping $script_name (ignored)" >> "$LOGFILE"
            continue
        fi

        # Execute script
        if [[ -x "$script" ]]; then
            if [[ "$RUN_AS_ROOT" == "yes" ]]; then
                echo "🔧 Running (root) $script_name..." >> "$LOGFILE"
                echo "qwerty" | sudo -S /bin/bash "$script" >> "$LOGFILE" 2>&1
            else
                echo "▶ Running (user) $script_name..." >> "$LOGFILE"
                /bin/bash "$script" >> "$LOGFILE" 2>&1 &
            fi
        else
            echo "⚠️ Skipping $script_name (not executable)" >> "$LOGFILE"
        fi
    done
}

# Run root scripts first
run_scripts "$ROOT_SCRIPTS_DIR" "yes"

# Run user scripts
run_scripts "$USER_SCRIPTS_DIR" "yes"

echo "✅ Finished executing startup scripts at $(date)" >> "$LOGFILE"
