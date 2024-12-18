### Description: Script to generate datasets for the CARLA vehicle simulation project.

## Set the script to fail if any command fails
set -e

## Convert the Jupyter notebooks to scripts
jupyter nbconvert \
    --to script run_simulation.ipynb
jupyter nbconvert \
    --to script run_simulation_postprocessing.ipynb

# Check if the CARLA simulator is running
echo "Checking if the CARLA simulator is running..."
if pgrep -f "CarlaUE4" > /dev/null
then
    echo "CARLA simulator is running. Proceeding with the dataset generation."
else
    echo "CARLA simulator is not running. Please start the simulator and try again."
    exit 1
fi

## Generate simulation data for the different towns

# Town3 small route
# python3 run_simulation.py \
#     --ego_vehicle_extrinsics '/home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf' \
#     --ego_vehicle_intrinsics '/home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_intrinsics.json' \
#     --episode_config '/home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/town03.smallpath.json' \
#     --output_dir '/media/leppsalu/SSD_Storage/generated_data_town03_small' \
#     --skip_validation

# python3 run_simulation_postprocessing.py \
#     --ego_vehicle_extrinsics /home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf \
#     --ego_vehicle_intrinsics /home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_intrinsics.json \
#     --input_dir /media/leppsalu/SSD_Storage/generated_data_town03_small \
#     --output_dir /media/leppsalu/SSD_Storage/processed_data_town03_small \
#     --batch_size 100

# Town3 full route
# python3 run_simulation.py \
#     --ego_vehicle_extrinsics '/home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf' \
#     --ego_vehicle_intrinsics '/home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_intrinsics.json' \
#     --episode_config '/home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/town03.path.json' \
#     --output_dir '/media/leppsalu/SSD_Storage/generated_data_town03' \
#     --skip_validation

python3 run_simulation_postprocessing.py \
    --ego_vehicle_extrinsics /home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf \
    --ego_vehicle_intrinsics /home/leppsalu/Desktop/Github/voxel-visibility/CARLA-vehicle-simulation/src/config/carla_intrinsics.json \
    --input_dir /media/leppsalu/SSD_Storage/generated_data_town03 \
    --output_dir /media/leppsalu/SSD_Storage/processed_data_town03 \
    --batch_size 100 \
    --process_from_frame 10380 \
    --mode 'minimal'