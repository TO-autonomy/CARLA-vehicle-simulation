import os

def rename_folders(target_folder, folder_mappings):
    """
    Renames subfolders in the target_folder based on the folder_mappings dictionary.

    Parameters:
    - target_folder: The path to the target folder where renaming should occur.
    - folder_mappings: A dictionary mapping original folder names to new folder names.
    """
    for root, dirs, files in os.walk(target_folder):
        for dir_name in dirs:
            # Check if the current directory name is in the mappings
            if dir_name in folder_mappings:
                # Create the old and new path
                old_folder_path = os.path.join(root, dir_name)
                new_folder_name = folder_mappings[dir_name]
                new_folder_path = os.path.join(root, new_folder_name)

                # Rename the directory
                os.rename(old_folder_path, new_folder_path)
                print(f'Renamed: "{old_folder_path}" to "{new_folder_path}"')

original_folder_names = [
    "AGV_sensors_cameras_front_raw", "AGV_sensors_cameras_front_left_raw", "AGV_sensors_cameras_front_right_raw", 
    "AGV_sensors_cameras_back_raw", "AGV_sensors_cameras_back_left_raw", "AGV_sensors_cameras_back_right_raw", 
    "AGV_sensors_lidar_top_raw"
]

new_folder_names = ["CAM_FRONT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT", "LIDAR_TOP"]

folder_names = dict()
for original_name, new_name in zip(original_folder_names, new_folder_names):
    folder_names[original_name] = new_name

print(folder_names)
# Call the function with your target folder and mappings
rename_folders('/home/leppsalu/Desktop/CARLA-vehicle-simulation/src/processed_data_town10_0-14_checkpoints', folder_names)