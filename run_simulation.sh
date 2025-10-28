#!/bin/bash

## Set the script to fail if any command fails
set -e

## Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator
INPUT_FILE=${1:-$SOURCE_DIR/config/scenarios/town10.scenario1.toml} # Default scenario file if none provided

echo "========================================"
echo "CARLA vehicle simulation"
echo "========================================"

# Add CARLA PythonAPI to PYTHONPATH
export PYTHONPATH="$PYTHONPATH:$SIMULATOR_DIR/PythonAPI/carla"

# Start CARLA server
bash $CURRENT_DIR/start_simulator.sh

echo "==="

# Determine the file type and run the appropriate simulation script
FILE_EXTENSION="${INPUT_FILE##*.}"
if [ "$FILE_EXTENSION" = "toml" ]; then
    python3 $SOURCE_DIR/run_planned_simulation.py \
        --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
        --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
        --scenario $INPUT_FILE \
        --output_dir $CURRENT_DIR/generated_data \
        --skip_validation
elif [ "$FILE_EXTENSION" = "rec" ]; then
    # Run simulation to generate dataset
    python3 $SOURCE_DIR/run_recorded_simulation.py \
        --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
        --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
        --recording $INPUT_FILE \
        --output_dir $CURRENT_DIR/generated_data
else
    echo "Error: Unsupported file type. Please provide a .toml or .rec file."
    exit 1
fi

# # Run post processing of simulation data to create a dataset (uncomment commands below to run)
# python3 $SOURCE_DIR/run_simulation_postprocessing.py\
#     --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
#     --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
#     --input_dir $CURRENT_DIR/generated_data \
#     --output_dir $CURRENT_DIR/processed_data \
#     --mode minimal

# # Run visualization of the generated simulation data (uncomment command below to run)
# python3 $SOURCE_DIR/run_simulation_visualization.py \
#     --input_dir $CURRENT_DIR/generated_data \
#     --layout "3-1" \
#     --sensor_subdirs "CAM_FRONT_LEFT,CAM_FRONT,CAM_FRONT_RIGHT,LIDAR_TOP" \
#     --output_dir $CURRENT_DIR/generated_data/visualizations

echo "Simulation pipeline completed."
echo "Exiting..."
