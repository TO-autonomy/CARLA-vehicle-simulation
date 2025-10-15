### Description: Script to generate datasets for the CARLA vehicle simulation project.
echo "========================================"
echo "CARLA (recorded) vehicle simulation"
echo "========================================"

## Set the script to fail if any command fails
set -e

## Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator

# Add CARLA PythonAPI to PYTHONPATH
export PYTHONPATH="$PYTHONPATH:$SIMULATOR_DIR/PythonAPI/carla"

# Start CARLA server
sh $CURRENT_DIR/start_simulator.sh $SIMULATOR_DIR/CarlaUE4.sh

echo "==="

# Run simulation to generate dataset
python3 $SOURCE_DIR/run_recorded_simulation.py \
    --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
    --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
    --recording $CURRENT_DIR/recording.rec \
    --output_dir $SOURCE_DIR/generated_data

# ## Run post processing of simulation data to create a dataset (uncomment commands below to run)
# POSTPROCESSING_SCRIPT_PY=$SOURCE_DIR/run_simulation_postprocessing.py
# python3 $POSTPROCESSING_SCRIPT_PY \
#     --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
#     --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
#     --input_dir $SOURCE_DIR/generated_data \
#     --output_dir $SOURCE_DIR/processed_data \
#     --batch_size 20

# python3 $SOURCE_DIR/run_simulation_visualization.py \
#     --source_dir $SOURCE_DIR/generated_data \
#     --sensor_subdirs CAM_FRONT_LEFT, CAM_FRONT, CAM_FRONT_RIGHT, LIDAR_TOP \
#     --output_dir $SOURCE_DIR/generated_data_visualizations # \
#     # --output_video $SOURCE_DIR/visualizations/simulation_video.mp4
