# Create a script that can be run using the following command:
# python3 $SOURCE_DIR/run_simulation_visualization.py \
#     --input_dir $SOURCE_DIR/generated_data \
#     --sensor_subdirs CAM_FRONT_LEFT CAM_FRONT CAM_FRONT_RIGHT LIDAR_TOP \
#     --frame_rate 10 \
#     --layout 3x1 \
#     --output_dir $SOURCE_DIR/generated_data_visualizations # \
#     --output_video $SOURCE_DIR/visualizations/simulation_video.mp4
#
# The script should read the images from the specified sensor subdirectories within the input directory,
# arrange them in the specified layout (e.g., 3x1 for three images side by side), and save the resulting visualizations
# as images in the output directory. If an output video path is provided, the script should also compile the visualizations into a video file at the specified location.
# The script should handle different frame rates and ensure that the visualizations are synchronized across sensors.
# The script should also include error handling for missing files or directories and provide informative messages to the user.

import os
import cv2
import numpy as np
import argparse
from glob import glob
from pathlib import Path
from typing import List, Tuple
import open3d as o3d
from datetime import datetime

def parse_args():
    parser = argparse.ArgumentParser(description="Visualize multi-sensor data from CARLA simulation.")
    parser.add_argument('--input_dir', type=str, required=True, help='Directory containing sensor subdirectories with images.')
    parser.add_argument('--sensor_subdirs', type=str, required=True, help='Comma-separated list of sensor subdirectory names to visualize.')
    parser.add_argument('--layout', type=str, default='1', help='Layout for arranging images (e.g., "3-1-3" for rows with varying numbers of images).')
    parser.add_argument('--output_dir', type=str, required=False, help='Directory to save the visualizations.')
    parser.add_argument('--output_video', type=str, default=None, help='Path to save the output video file (optional).')
    parser.add_argument('--video_frame_rate', type=int, default=10, help='Frame rate for visualization and video output.')
    parser.add_argument('--video_resolution', type=str, default="1920x1080", help='Resolution of the output canvas (width and height).')
    parser.add_argument('--skip_every_nth_frame', type=int, default=1, help='Skip every n-th frame for faster processing (default is 1, meaning no skipping).')
    return parser.parse_args()

def create_image_from_pcd(lidar_path: str, image_size: Tuple[int, int]=(600, 600)) -> np.ndarray:
    # Create a simple 2D projection of the point cloud for visualization by removing the z-axis
    # Use open3d to read the point cloud
    pcd = o3d.io.read_point_cloud(lidar_path)
    points = np.asarray(pcd.points)
    if points.size == 0:
        return np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8)
    def rasterize_to_bev(points, resolution=0.5, grid_size=25):
        # Rasterize 3D points to a BEV occupancy map
        bev_map = np.zeros((int(grid_size), int(grid_size)))
        grid_coords = np.floor(points[:, :2] / resolution).astype(np.int32) + int(grid_size // 2)
        valid_points = (grid_coords[:, 0] >= 0) & (grid_coords[:, 0] < bev_map.shape[0]) & \
                    (grid_coords[:, 1] >= 0) & (grid_coords[:, 1] < bev_map.shape[1])
        bev_map[grid_coords[valid_points, 1], grid_coords[valid_points, 0]] = 255
        bev_map = np.flip(bev_map, axis=0)
        return bev_map
    img = rasterize_to_bev(points, grid_size=min(image_size))
    img = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    img = cv2.resize(img, image_size)
    return img

def arrange_images(images: List[np.ndarray], layout: List[int], canvas_resolution: str) -> np.ndarray:
    row_images = []
    idx = 0
    for row_count in layout:
        row = images[idx:idx + row_count]
        idx += row_count

        # Resize images in the row to the same height
        min_height = min(img.shape[0] for img in row)
        resized_row = [cv2.resize(img, (int(img.shape[1] * min_height / img.shape[0]), min_height)) for img in row]

        # Concatenate images horizontally
        row_images.append(cv2.hconcat(resized_row))

    # Determine the width of the canvas
    max_width = max(row.shape[1] for row in row_images)

    # Pad rows to the same width and concatenate vertically
    padded_rows = [cv2.copyMakeBorder(row, 0, 0, (max_width - row.shape[1]) // 2, (max_width - row.shape[1] + 1) // 2, cv2.BORDER_CONSTANT, value=(0, 0, 0)) for row in row_images]
    canvas = cv2.vconcat(padded_rows)

    # Resize canvas to desired resolution
    height, width = canvas.shape[:2]
    new_width, new_height = map(int, canvas_resolution.split('x'))
    scale = min(new_width / width, new_height / height)
    new_width = int(width * scale)
    new_height = int(height * scale)
    canvas = cv2.resize(canvas, (new_width, new_height))
    return canvas

def annotate_image(image: np.ndarray, text: str) -> np.ndarray:
    annotated_image = image.copy()
    if len(text) == 0:
        return annotated_image
    elif len(text) > 20:
        text = text[:17] + '...'
    cv2.putText(annotated_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                1, (0, 255, 0), 2, cv2.LINE_AA)
    return annotated_image

def get_filenames_from_paths(paths: List[str]) -> List[str]:
    return [Path(p).name for p in paths]

if __name__ == "__main__":
    args = parse_args()
    
    sensor_subdirs = args.sensor_subdirs.split(',')

    filenames = set()
    for subdir in sensor_subdirs:
        subdir_path = os.path.join(args.input_dir, subdir)
        if not os.path.isdir(subdir_path):
            raise FileNotFoundError(f"Sensor subdirectory {subdir_path} does not exist.")
        files = glob(os.path.join(subdir_path, '*'))
        filenames.update(get_filenames_from_paths(files))
    print(f"Found {len(filenames)} unique files across specified sensor subdirectories.")
    timestamps = sorted([int(filename.split(".")[0]) for filename in filenames if filename.split(".")[0].isdigit()])
    print(len(timestamps), "timestamps found.")

    layout_parts = args.layout.split('-')
    if not all(part.isdigit() for part in layout_parts):
        raise ValueError("Layout must be in the format 'N-M-O', e.g., '3-1-3'.")
    layout = [int(part) for part in layout_parts]
    if sum(layout) != len(sensor_subdirs):
        raise ValueError(f"Layout {layout} is incompatiple with number of sensors ({len(sensor_subdirs)}).")
    
    prev_images = []
    for i, timestamp in enumerate(timestamps):
        if i % 100 == 0:
            print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Processing frame {i+1}/{len(timestamps)}")
            

        images = []
        for j, subdir in enumerate(sensor_subdirs):
            subdir_path = os.path.join(args.input_dir, subdir)
            img_path = os.path.join(subdir_path, f"{timestamp:06d}")
            if os.path.isfile(img_path + '.png'):
                img = cv2.imread(img_path + '.png')
            elif os.path.isfile(img_path + '.ply'):
                img = create_image_from_pcd(img_path + '.ply')
            else:
                try:
                    # If one sensor is missing, use a blank image
                    print(f"Warning: No image or point cloud found for timestamp {timestamp} in {subdir_path}. Using blank image.")
                    img = np.zeros_like(prev_images[j])
                except IndexError:
                    print(f"Error: No previous images to infer size for blank image at timestamp {timestamp} in {subdir_path}. Exiting...")
                    exit(1)
            img = annotate_image(img, f"{subdir}")
            images.append(img)
            prev_images = images.copy()
        
        combined_image = arrange_images(images, layout, args.video_resolution)

        if args.output_dir:
            os.makedirs(args.output_dir, exist_ok=True)
            output_image_path = os.path.join(args.output_dir, f"{timestamp:06d}.png")
            cv2.imwrite(output_image_path, combined_image)
        elif args.output_dir and i == len(timestamps) - 1:
            print(f"Saved visualizations to {args.output_dir}")
        
        if args.output_video and i == 0:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(args.output_video, fourcc, args.video_frame_rate, 
                                           (combined_image.shape[1], combined_image.shape[0]))
        elif args.output_video:
            video_writer.write(combined_image)
        elif args.output_video and i == len(timestamps) - 1:
            print(f"Saving video to {args.output_video}")
            video_writer.release()
    
    print("Visualization completed.")

