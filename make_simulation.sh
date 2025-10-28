#!/bin/bash

## Set the script to fail if any command fails
set -e

## Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator
OUTPUT_FILE=${1:-$SOURCE_DIR/config/scenarios/new_scenario.toml} # Default scenario file if none provided

echo "========================================"
echo "CARLA simulation maker"
echo "========================================"

# Start CARLA server
sh $CURRENT_DIR/start_simulator.sh

echo "==="

# Check if output file has .toml or .rec extension and
# run the appropriate script
FILE_EXTENSION="${OUTPUT_FILE##*.}"
if [ "$FILE_EXTENSION" = "toml" ]; then
    # Load CARLA map and wait for it to be ready
    python3 $SOURCE_DIR/tools/create_simulation_scenario.py \
        --output_file $OUTPUT_FILE
elif [ "$FILE_EXTENSION" = "rec" ]; then
    python3 $SOURCE_DIR/tools/create_simulation_recording.py \
        --output_file $OUTPUT_FILE
else
    echo "Error: Unsupported file type. Please provide a .toml or .rec file extension."
    exit 1
fi

