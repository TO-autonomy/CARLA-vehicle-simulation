echo "========================================"
echo "CARLA vehicle simulation"
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
# sh $CURRENT_DIR/start_simulator.sh $SIMULATOR_DIR/CarlaUE4.sh

# echo "==="

# # Run simulation to generate dataset
# python3 $SOURCE_DIR/run_simulation.py \
#     --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
#     --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
#     --scenario $SOURCE_DIR/config/scenarios/town03.scenario1.toml \
#     --output_dir $SOURCE_DIR/generated_data \
#     --skip_validation

## Run post processing of simulation data to create a dataset (uncomment commands below to run)
POSTPROCESSING_SCRIPT_PY=$SOURCE_DIR/run_simulation_postprocessing.py
python3 $POSTPROCESSING_SCRIPT_PY \
    --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
    --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
    --input_dir /media/leppsalu/SSD_Storage/generated_data_mine01_collection/generated_data_mine01-20251001-freedrive1 \
    --output_dir /media/leppsalu/SSD_Storage/generated_data_mine01_collection/processed_data_mine01-20251001-freedrive1 \
    --batch_size 20 \
    --mode debug

# python3 $SOURCE_DIR/run_simulation_visualization.py \
#     --source_dir $SOURCE_DIR/generated_data \
#     --sensor_subdirs CAM_FRONT_LEFT, CAM_FRONT, CAM_FRONT_RIGHT, LIDAR_TOP \
#     --output_dir $SOURCE_DIR/generated_data_visualizations # \
#     # --output_video $SOURCE_DIR/visualizations/simulation_video.mp4
