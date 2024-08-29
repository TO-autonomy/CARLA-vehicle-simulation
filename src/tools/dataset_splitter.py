import os
import shutil

def copy_files_within_timestamp_range(source_dir, target_dir, start_timestamp, end_timestamp):
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            filename, extension = os.path.splitext(file)
            if filename.isdigit() and start_timestamp <= int(filename) <= end_timestamp:
                source_path = os.path.join(root, file)
                target_path = os.path.join(target_dir, os.path.relpath(source_path, source_dir))
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                shutil.copy2(source_path, target_path)

# This program extracts a subset of the data from the source directory 
# and copies it to the target directory. 

# Example usage
source_dir = '/home/erkoiv/Desktop/CARLA-vehicle-simulation/src/generated_data'
target_dir = '/home/erkoiv/Desktop/CARLA-vehicle-simulation/src/generated_data_small'
start_timestamp = 6451600000 
end_timestamp = 7551616592
copy_files_within_timestamp_range(source_dir, target_dir, start_timestamp, end_timestamp)