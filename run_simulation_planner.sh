echo "========================================"
echo "CARLA simulation scenario planner"
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

# Load CARLA map and wait for it to be ready
python3 $SOURCE_DIR/tools/create_simulation_scenario.py 

echo "Scenario created."
echo "Exiting..."

