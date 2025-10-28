#!/bin/bash

# Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator
SERVER_SCRIPT="$SIMULATOR_DIR/CarlaUE4.sh"
SERVER_SCRIPT_BASENAME="CarlaUE4"
SERVER_BOOT_TIME=10 # Time to wait for the server to boot up (in seconds); Can be adjusted based on system performance.


echo "Checking if the CARLA simulator is running..."
SERVER_PID=$(pgrep -af "$SERVER_SCRIPT_BASENAME" | grep -v "$0" | awk '{print $1}')
if [ -z "$SERVER_PID" ]; then
    echo "CARLA simulator is not running."
    echo "Starting server..."
    # Start server script and redirect only stdout to /dev/null
    bash "$SERVER_SCRIPT" > /dev/null &
    sleep $SERVER_BOOT_TIME
else
    echo "CARLA simulator is already running. (PID: $SERVER_PID)"
    read -p "Do you want to restart the CARLA simulator? (y/n): " RESTART_CARLA
    if [ "$RESTART_CARLA" = "y" ]; then
        echo "Restarting CARLA simulator..."
        kill -9 "$SERVER_PID"
        sleep 2
        # Start server script and redirect only stdout to /dev/null
        bash "$SERVER_SCRIPT" > /dev/null &
        sleep $SERVER_BOOT_TIME
    fi
fi

if ! pgrep -af "$SERVER_SCRIPT_BASENAME" | grep -v "$0" > /dev/null; then
    echo "Failed to start CARLA simulator. Exiting..."
    exit 1
fi

echo "CARLA simulator is running."