#!/bin/bash

# Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator
SERVER_SCRIPT="$SIMULATOR_DIR/CarlaUE4.sh"
SERVER_SCRIPT_BASENAME=$(basename "$SERVER_SCRIPT")

echo "Checking if the CARLA simulator is running..."
if ! pgrep -af "$SERVER_SCRIPT_BASENAME" | grep -v "$0" > /dev/null; then
    echo "CARLA simulator is not running."
    echo "Starting server..."
    # Start server script and redirect only stdout to /dev/null
    sh "$SERVER_SCRIPT" > /dev/null &
    sleep 5
    if ! pgrep -af "$SERVER_SCRIPT_BASENAME" | grep -v "$0" > /dev/null; then
        echo "Failed to start CARLA simulator. Exiting..."
        exit 1
    fi
fi

echo "CARLA simulator is running."