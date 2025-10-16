echo "========================================"
echo "CARLA simulation recorder"
echo "========================================"

## Set the script to fail if any command fails
set -e

## Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator

# Start CARLA server
sh $CURRENT_DIR/start_simulator.sh

echo "==="

# Run manual control script to create a new recording
python3 $SOURCE_DIR/tools/create_simulation_recording.py

echo "Simulation recording completed."
echo "Exiting..."
