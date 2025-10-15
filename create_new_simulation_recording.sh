## Define simulation environment variables
RECORDING_PATH=$(pwd)"/recording.rec" ##$(pwd)"/recording_$(date +%Y%m%d_%H%M%S).rec"
MAP="Town10HD"
FPS=10
WEATHER="ClearNoon"
AI_VEHICLES=2
DORMANT_VEHICLES=5
AI_PEDESTRIANS=2
DORMANT_PEDESTRIANS=5


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
sh $CURRENT_DIR/start_simulator.sh $SIMULATOR_DIR/CarlaUE4.sh

echo "==="

# Load CARLA map and wait for it to be ready
python3 $SOURCE_DIR/tools/config.py --map $MAP --weather $WEATHER --fps $FPS > /dev/null

# Run manual control script to create a new recording
python3 $SOURCE_DIR/tools/create_simulation_recording.py \
    --recording $RECORDING_PATH \
    --sync \
    --filter "vehicle.dodge.charger_2020" \
    --ai_vehicles $AI_VEHICLES \
    --ai_pedestrians $AI_PEDESTRIANS \
    --dormant_vehicles $DORMANT_VEHICLES \
    --dormant_pedestrians $DORMANT_PEDESTRIANS

echo "Recording saved to: $RECORDING_PATH"
echo "Exiting..."
