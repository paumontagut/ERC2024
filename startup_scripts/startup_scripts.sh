#!/bin/bash
SCRIPT_DIR="/home/ujiroboticsteam/ERC2024/startup_scripts/scripts"
LOGFILE="/home/ujiroboticsteam/ERC2024/startup_scripts/startup_scripts.log"

echo "Executing startup scripts from $SCRIPT_DIR at $(date)" >> $LOGFILE

# Execute all .sh scripts in the folder
for script in "$SCRIPT_DIR"/*.sh; do
    if [ -x "$script" ]; then
        echo "Running $script..." >> $LOGFILE
        /bin/bash "$script" >> $LOGFILE 2>&1 &
    else
        echo "Skipping $script (not executable)" >> $LOGFILE
    fi
done

echo "Finished executing startup scripts at $(date)" >> $LOGFILE
