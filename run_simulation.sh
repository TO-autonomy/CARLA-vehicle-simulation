### Description: Script to generate datasets for the CARLA vehicle simulation project.
echo "========================================"
echo "CARLA vehicle simulation v0.1.0"
echo "========================================"

## Set the script to fail if any command fails
set -e

## Define the paths to the source and simulator directories
CURRENT_DIR=$(pwd)
SOURCE_DIR=$CURRENT_DIR/src
SIMULATOR_DIR=$SOURCE_DIR/CARLASimulator

## Check if current directory is the root of the project by checking the existence of the SOURCE_DIR and SIMULATOR_DIR directory
if [ ! -d "$SOURCE_DIR" ] || [ ! -d "$SIMULATOR_DIR" ]; then
    echo "Please run this script from the root of the project."
    exit 1
fi

## Add CARLA PythonAPI to environment path variables (if necessary)
SIMULATOR_PYTHON_API=$SIMULATOR_DIR/PythonAPI/carla
if [ ! -d "$SIMULATOR_PYTHON_API" ]; then
    echo "PythonAPI ($SIMULATOR_PYTHON_API) is missing from the CARLA simulator source directory ($SIMULATOR_DIR). Please check the CARLA installation and ensure that PythonAPI is available."
    exit 1
fi
export PYTHONPATH="$PYTHONPATH:$SIMULATOR_PYTHON_API"


## Check if the CARLA simulator is running, if not, start the server
SERVER_SCRIPT=$SIMULATOR_DIR/CarlaUE4.sh
echo "Checking if the CARLA simulator is running..."
if ! pgrep -f "CarlaUE4" > /dev/null
then
    echo "CARLA simulator is not running. Starting server..."
    sh $SERVER_SCRIPT &
    sleep 3
    if ! pgrep -f "CarlaUE4" > /dev/null
    then
        echo "Failed to start CARLA simulator. Exiting..."
        exit 1
    fi
fi
echo "CARLA simulator is running. Proceeding with the dataset generation."

## Convert source file to python script and run simulation
SIMULATION_SCRIPT_IPYNB=$SOURCE_DIR/run_simulation.ipynb
SIMULATION_SCRIPT_PY=$SOURCE_DIR/run_simulation.py
jupyter nbconvert \
    --to script $SIMULATION_SCRIPT_IPYNB --log-level=CRITICAL
python3 $SIMULATION_SCRIPT_PY \
    --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
    --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
    --episode_config $SOURCE_DIR/config/routes/town03.path.json \
    --output_dir $SOURCE_DIR/generated_data \
    --skip_validation

## Convert source file to python script and run post processing of simulation data (uncomment commands below to run)
# POSTPROCESSING_SCRIPT_IPYNB=$SOURCE_DIR/run_simulation_postprocessing.ipynb
# POSTPROCESSING_SCRIPT_PY=$SOURCE_DIR/run_simulation_postprocessing.py
# jupyter nbconvert \
#     --to script $POSTPROCESSING_SCRIPT_IPYNB --log-level=CRITICAL
# python3 $POSTPROCESSING_SCRIPT_PY \
#     --ego_vehicle_extrinsics $SOURCE_DIR/config/carla_extrinsics.urdf \
#     --ego_vehicle_intrinsics $SOURCE_DIR/config/carla_intrinsics.json \
#     --input_dir $SOURCE_DIR/generated_data \
#     --output_dir $SOURCE_DIR/processed_data \
#     --batch_size 20 \
#     --mode 'debug'