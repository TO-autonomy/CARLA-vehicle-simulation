import cv2
import numpy as np
import open3d as o3d
import os
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from scipy.ndimage import distance_transform_edt
import json
import shutil
import argparse
import sys
from datetime import datetime

import concurrent.futures
from queue import Queue
from threading import Lock
from joblib import Parallel, delayed

from classes.CARLASemantics import SemanticTags, SemanticColors
from classes.carla.Sensors import Camera
from classes.util.URDFParser import URDFParser
from classes.util.Viewshed3D import ViewShed3D, compute_camera_matrix_4x4


parser = argparse.ArgumentParser(description='Run simulation postprocessing.')
parser.add_argument('--ego_vehicle_extrinsics', type=str, required=True, help='Path to the ego vehicle extrinsics file')
parser.add_argument('--ego_vehicle_intrinsics', type=str, required=True, help='Path to the ego vehicle intrinsics file')
parser.add_argument('--input_dir', type=str, required=True, help='Path to the directory with simulation data')
parser.add_argument('--output_dir', type=str, required=True, help='Path to the output directory where the processed data will be saved')
parser.add_argument('--batch_size', type=int, default=15, help='Number of frames to process in a single batch (default: 15)')
parser.add_argument('--n_workers', type=int, default=-1, help='Number of workers for multiprocessing (default: all cores)')
parser.add_argument('--n_frames_per_bag', type=int, default=1800, help='Number of frames in a bag (default: 1800)')
parser.add_argument('--from_timestamp', type=int, default=None, help='Start post processing from frame with this timestamp')
parser.add_argument('--to_timestamp', type=int, default=None, help='End post processing at frame with this timestamp')
parser.add_argument('--mode', choices=['minimal', 'full', 'debug'], default='debug', help="Processing mode: 'minimal', 'full', or 'debug' (default: 'debug')")
parser.add_argument('--clean_input_dir', action='store_true', help='Clean input directory before processing (removes files from previous runs)')
args = parser.parse_args()

SOURCE_DIR = args.input_dir
TARGET_DIR = args.output_dir 
EXTRINSICS_FILEPATH = args.ego_vehicle_extrinsics
INTRINSICS_FILEPATH = args.ego_vehicle_intrinsics
N_FRAMES_PER_BAG = args.n_frames_per_bag
BATCH_SIZE = args.batch_size
N_WORKERS = args.n_workers
START_FROM_TIMESTAMP = args.from_timestamp 
END_AT_TIMESTAMP = args.to_timestamp
DATA_PROCESING_MODE = args.mode
CLEAN_SOURCE_DIR = args.clean_input_dir 

# Directory names for different sensor data
LIDAR_DIR = "LIDAR_TOP"
CAM_DIRS = ["CAM_FRONT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"]
SEMANTIC_CAM_DIRS =  ["SEMANTIC_CAM_FRONT", "SEMANTIC_CAM_FRONT_LEFT", "SEMANTIC_CAM_FRONT_RIGHT", "SEMANTIC_CAM_BACK", "SEMANTIC_CAM_BACK_LEFT", "SEMANTIC_CAM_BACK_RIGHT"]
DEPTH_CAM_DIRS = ["DEPTH_CAM_FRONT", "DEPTH_CAM_FRONT_LEFT", "DEPTH_CAM_FRONT_RIGHT", "DEPTH_CAM_BACK", "DEPTH_CAM_BACK_LEFT", "DEPTH_CAM_BACK_RIGHT"]
DEPTH_BEV_DIR = "DEPTH_BEV"
DEPTH_VISIBILITY_DIR = "DEPTH_VISIBILITY"

# Grid and BEV map parameters
GRID_SIZE = 104         # Output grid size (pixels)
GRID_RESOLUTION = 0.5   # Output grid resolution (meters)
GRID_ORIGIN = np.array([GRID_SIZE // 2, GRID_SIZE // 2]) 
GRID_DIAGONAL = np.sqrt(GRID_SIZE**2 + GRID_SIZE**2)
MAX_POSTPROCESSING_DISTANCE = GRID_RESOLUTION * np.ceil(GRID_DIAGONAL / 2) # Max distance for processing

# Load sensor extrinsics and intrinsics
EXTRINSICS = URDFParser(EXTRINSICS_FILEPATH)
with open(INTRINSICS_FILEPATH, "r") as INTRINSICS_FILE:
    INTRINSICS = json.load(INTRINSICS_FILE)

def get_intrinsics_dict(query_name):
    # Retrieve intrinsics for a given sensor, handling depth/semantic camera naming
    sensor_name = query_name
    if "DEPTH_" in sensor_name:
        sensor_name = sensor_name.replace("DEPTH_", "")
    if "SEMANTIC_" in sensor_name:
        sensor_name = sensor_name.replace("SEMANTIC_", "")
    return INTRINSICS[sensor_name]

def get_intrinsics_matrix(query_name):
    # Build camera intrinsics matrix for projection
    sensor_name = query_name
    if "DEPTH_" in sensor_name:
        sensor_name = sensor_name.replace("DEPTH_", "")
    if "SEMANTIC_" in sensor_name:
        sensor_name = sensor_name.replace("SEMANTIC_", "")
    camera_intrinsics = INTRINSICS[sensor_name]
    fx = camera_intrinsics.get('fx', camera_intrinsics.get('fl'))
    fy = camera_intrinsics.get('fy', camera_intrinsics.get('fl'))
    w, h = camera_intrinsics.get('w'), camera_intrinsics.get('h')
    ppx = w / 2
    ppy = h / 2
    intrinsics_matrix = np.array([[fx, 0, ppx, 0],
                                [0, fy, ppy, 0],
                                [0, 0, 1, 0]], dtype=int)
    return intrinsics_matrix

def get_extrinsics_matrix(query_name):
    # Get transformation matrix from root to sensor
    sensor_name = query_name
    if "DEPTH_" in sensor_name:
        sensor_name = sensor_name.replace("DEPTH_", "")
    if "SEMANTIC_" in sensor_name:
        sensor_name = sensor_name.replace("SEMANTIC_", "")
    robot = EXTRINSICS.robot
    root_link = EXTRINSICS.root
    extrinsics_matrix = EXTRINSICS.compute_chain_transform(robot.get_chain(root_link, sensor_name))
    return extrinsics_matrix

# Filesystem utility functions
def remove_files_in_dir(data_dir, string_to_find):
    # Remove files containing a specific substring in their name
    for root, dirs, files in os.walk(data_dir):
        for file in files:
            if string_to_find in file:
                file_path = os.path.join(root, file)
                try:
                    os.remove(file_path)
                except Exception as e:
                    print(f"Error removing {file_path}: {e}")

def get_all_filenames(dir, no_extension=False):
    # List all filenames in a directory, optionally without extension
    if no_extension:
        return [filename.split(".")[0] for filename in os.listdir(dir)]
    return [filename for filename in os.listdir(dir)]

def clean_up_lidar_dir():
    # Remove intermediate files from LIDAR directory
    LIDAR_DIR_PATH = os.path.join(SOURCE_DIR, LIDAR_DIR)
    remove_files_in_dir(LIDAR_DIR_PATH, ".bev.")
    remove_files_in_dir(LIDAR_DIR_PATH, ".ground.")

def clean_up_camera_dirs():
    # Remove intermediate files from camera directories
    for CAM_DIR in CAM_DIRS:
        CAM_DIR_PATH = os.path.join(SOURCE_DIR, CAM_DIR)
        remove_files_in_dir(CAM_DIR_PATH, ".pointcloud.")
        remove_files_in_dir(CAM_DIR_PATH, ".visibility.")
        remove_files_in_dir(CAM_DIR_PATH, ".fov.")

def clean_up_depth_camera_dirs():
    # Remove intermediate files from depth camera directories
    for DEPTH_CAM_DIR in DEPTH_CAM_DIRS:
        DEPTH_CAM_DIR_PATH = os.path.join(SOURCE_DIR, DEPTH_CAM_DIR)
        remove_files_in_dir(DEPTH_CAM_DIR_PATH, ".ply")
        remove_files_in_dir(DEPTH_CAM_DIR_PATH, ".fov.")
        remove_files_in_dir(DEPTH_CAM_DIR_PATH, ".visibility.")

def clean_up_depth_bev_dir():
    # Remove all files from depth BEV directory
    DEPTH_BEV_DIR_PATH = os.path.join(SOURCE_DIR, DEPTH_BEV_DIR)
    remove_files_in_dir(DEPTH_BEV_DIR_PATH, ".")

def clean_up_depth_visibility_dir():
    # Remove all files from depth visibility directory
    DEPTH_VISIBILITY_DIR_PATH = os.path.join(SOURCE_DIR, DEPTH_VISIBILITY_DIR)
    remove_files_in_dir(DEPTH_VISIBILITY_DIR_PATH, ".")
        
def save_point_cloud(file_path, point_cloud):
    # Save Open3D point cloud to file
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)
    if type(point_cloud) is o3d.geometry.PointCloud:
        o3d.io.write_point_cloud(file_path, point_cloud)
        return
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)
    o3d.io.write_point_cloud(file_path, pcd)

def read_point_cloud(file_path):
    # Read point cloud from file and return as numpy array
    point_cloud = o3d.io.read_point_cloud(file_path)
    return np.asarray(point_cloud.points)

def save_image(file_path, mask):
    # Save image to file
    directory = os.path.dirname(file_path)
    if not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)
    cv2.imwrite(file_path, mask)

# Image and depth map utilities
def read_image(file_path, color_format="BGR"):
    # Read image from file in specified color format
    image = None
    if color_format == "BGR":
        image = cv2.imread(file_path, cv2.IMREAD_COLOR)
    elif color_format == "RGB":
        image = plt.imread(file_path)
    elif color_format == "RGBA":
        image = plt.imread(file_path, cv2.IMREAD_UNCHANGED)
    elif color_format == "GRAY":
        image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
    assert image is not None, f"Error reading image from {file_path}"
    return image

def calculate_depth_map_from_image(image):
    # Convert RGB-encoded depth image to depth map in meters
    array = image.astype(np.float32)
    normalized_depth = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0])
    normalized_depth /= 16777215.0
    meters_depth = 1000 * normalized_depth
    return meters_depth

def clip_depth_map(depth_map, clip_distance):
    # Clip depth values above a threshold
    depth_map[depth_map > clip_distance] = clip_distance
    return depth_map

def threshold_depth_map(depth_map, max_distance):
    # Zero out depth values above a threshold
    depth_map[depth_map > max_distance] = 0.0
    return depth_map

def create_o3d_pinhole_camera_intrinsics(camera_intrinsics):
    # Create Open3D camera intrinsics object from dictionary
    height, width = camera_intrinsics["h"], camera_intrinsics["w"]
    focal_length = camera_intrinsics["fl"]
    def calculate_fov(focal_length, image_width):
        fov_radians = 2 * np.arctan(image_width / (2 * focal_length))
        fov_degrees = np.degrees(fov_radians)
        return fov_degrees
    fov = calculate_fov(focal_length, width)
    fx = fy = 0.5 * width / np.tan(0.5 * np.radians(fov))
    cx = width / 2.0
    cy = height / 2.0
    o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    return o3d_intrinsic

def depth_map_to_point_cloud(depth_map, camera_intrinsics):
    # Convert depth map to Open3D point cloud
    height, width = camera_intrinsics["h"], camera_intrinsics["w"]
    focal_length = camera_intrinsics["fl"]
    def calculate_fov(focal_length, image_width):
        fov_radians = 2 * np.arctan(image_width / (2 * focal_length))
        fov_degrees = np.degrees(fov_radians)
        return fov_degrees
    fov = calculate_fov(focal_length, width)
    fx = fy = 0.5 * width / np.tan(0.5 * np.radians(fov))
    cx = width / 2.0
    cy = height / 2.0
    u, v = np.meshgrid(np.arange(depth_map.shape[1]), np.arange(depth_map.shape[0]))
    depth_map_points = np.zeros((depth_map.shape[0], depth_map.shape[1], 3), dtype=np.float32)
    Z = depth_map
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    depth_map_points[:, :, 0] = X
    depth_map_points[:, :, 1] = Y
    depth_map_points[:, :, 2] = Z
    depth_map_points = depth_map_points.reshape(-1, 3)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(depth_map_points)
    return point_cloud

def depth_image_to_point_cloud(depth_image, depth_camera_intrinsics, max_distance=None, clip_distance=None):
    # Convert depth image to point cloud, with optional clipping
    depth_map = calculate_depth_map_from_image(depth_image)
    if clip_distance is not None:
        depth_map = clip_depth_map(depth_map, clip_distance)
    if max_distance is not None:
        depth_map = threshold_depth_map(depth_map, max_distance)
    point_cloud = depth_map_to_point_cloud(depth_map, depth_camera_intrinsics)
    return point_cloud

def depth_image_to_semantic_point_cloud(depth_image, semantic_image, depth_camera_intrinsics, max_distance=None, clip_distance=None):
    # Convert depth and semantic images to colored point cloud
    point_cloud = depth_image_to_point_cloud(depth_image, depth_camera_intrinsics, max_distance, clip_distance)
    point_cloud_semantic_colors = semantic_image.reshape(-1, 3) / 255.0
    point_cloud.colors = o3d.utility.Vector3dVector(point_cloud_semantic_colors)
    return point_cloud

def get_points_from_semantic_point_cloud(point_cloud, semantic_tags=None):
    # Extract points with specified semantic tags from point cloud
    point_cloud_points = np.array(point_cloud.points)
    point_cloud_colors = (np.array(point_cloud.colors) * 255).astype(np.uint8) 
    point_cloud_semantic_tags = point_cloud_colors[:, 2]
    if semantic_tags is not None:
        semantic_tags = np.array(semantic_tags)
        point_cloud_points = point_cloud_points[np.where(np.isin(point_cloud_semantic_tags, semantic_tags))]
    return point_cloud_points

def get_ground_from_semantic_point_cloud(point_cloud):
    # Extract ground points from semantic point cloud
    ground_semantic_tags = np.array([ 
            SemanticTags.ROADLINE.value, SemanticTags.ROAD.value, 
            SemanticTags.SIDEWALK.value, SemanticTags.GROUND.value, 
            SemanticTags.WATER.value, SemanticTags.TERRAIN.value
        ], dtype=np.uint8)
    ground_points = get_points_from_semantic_point_cloud(point_cloud, ground_semantic_tags)
    ground_point_cloud = o3d.geometry.PointCloud()
    ground_point_cloud.points = o3d.utility.Vector3dVector(ground_points)
    return ground_point_cloud

def get_obstacles_from_semantic_point_cloud(point_cloud):
    # Extract obstacle points from semantic point cloud
    all_semantic_tags = [semantic_tag.value for semantic_tag in SemanticTags]
    ground_and_sky_semantic_tags = np.array([ 
            SemanticTags.ROADLINE.value, SemanticTags.ROAD.value, 
            SemanticTags.SIDEWALK.value, SemanticTags.GROUND.value, 
            SemanticTags.WATER.value, SemanticTags.TERRAIN.value
        ], dtype=np.uint8)
    obstacles_tags = np.setdiff1d(all_semantic_tags, ground_and_sky_semantic_tags)
    obstacle_points = get_points_from_semantic_point_cloud(point_cloud, obstacles_tags)
    obstacle_point_cloud = o3d.geometry.PointCloud()
    obstacle_point_cloud.points = o3d.utility.Vector3dVector(obstacle_points)
    return obstacle_point_cloud

def create_color_mask(image, colors, inverted=False):
    # Create a binary mask for specified colors in an image
    mask = np.full((image.shape[0], image.shape[1]), 0, dtype=np.uint8)
    B, G, R = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    for color in colors:
        r, g, b = color
        r_mask = R == r
        g_mask = G == g
        b_mask = B == b
        color_mask = r_mask & g_mask & b_mask
        mask[color_mask] = 255
    if inverted:
        mask = np.where(mask == 0, 255, 0).astype(np.uint8)
        return mask
    return mask

def mask_image(image, image_mask):
    # Apply a binary mask to an image
    masked_image = np.array(image)
    masked_image[~image_mask] = [0,0,0] 
    return masked_image

# FOV, visibility, and BEV utilities
def generate_mask(grid, camera_matrix, T, image_size):
    # Project grid points into camera image and return mask of visible points
    homogeneous_grid = np.vstack([grid[i].flatten() for i in range(3)] + [np.ones(grid[0].size)])
    tfd_points = np.dot(T, homogeneous_grid)
    tfd_points = np.dot(camera_matrix, tfd_points)
    mask_z = tfd_points[2] > 0
    tfd_points[2][tfd_points[2] == 0] = np.nan
    projected_points = tfd_points / tfd_points[2]
    mask = ((0 <= projected_points[0]) & (projected_points[0] < image_size[0]) &
            (0 <= projected_points[1]) & (projected_points[1] < image_size[1]) &
            mask_z)
    return mask.reshape(grid[0].shape)

def get_camera_fov_masks(camera_calibs, lidar_to_cam_tf_list=[], grid_size_m=50, resolution=0.5):
    # Compute FOV masks for all cameras on a BEV grid
    whole_mask = np.zeros((int(grid_size_m), int(grid_size_m)))
    x, y = np.meshgrid(np.arange(whole_mask.shape[1]), np.arange(whole_mask.shape[0]))
    x = x - whole_mask.shape[1] / 2
    y = y - whole_mask.shape[0] / 2
    z = np.zeros_like(x)
    bev_grid = np.array([x, y, z])
    bev_grid = np.expand_dims(bev_grid, axis=-1)
    cam_masks = {}
    for i, (camera, calib) in enumerate(camera_calibs.items()):
        intrinsic = calib['K']
        w, h = calib['w'], calib['h']
        if lidar_to_cam_tf_list:
            cam_T_lidar = np.linalg.inv(lidar_to_cam_tf_list[i])
        else:
            cam_T_lidar = np.linalg.inv(calib['T'])
        mask = generate_mask(bev_grid, camera_matrix=intrinsic, T=cam_T_lidar, image_size=(w, h))
        visible_bev = np.array([dim[mask] for dim in bev_grid])
        mask_ref = visible_bev[:2]
        cam_mask = np.zeros_like(mask)
        mask_ref[1] += mask.shape[1] / 2
        mask_ref[0] += mask.shape[0] / 2
        mask_ref = np.round(mask_ref).astype(int)
        cam_mask[mask_ref[1], mask_ref[0]] = 1
        cam_mask = cam_mask.squeeze(-1)
        cam_mask = np.flip(cam_mask, axis=0)
        cam_masks[camera] = cam_mask
    return cam_masks

def get_fov_mask(image, transformation_matrix, camera_intrinsics_matrix):
    # Generate FOV mask for a single camera/image
    camera_calibs = {
        'camera': {
            'K': camera_intrinsics_matrix,
            'w': image.shape[1],
            'h': image.shape[0],
            'T': transformation_matrix
        }
    }
    cam_masks = get_camera_fov_masks(camera_calibs, grid_size_m=GRID_SIZE, resolution=GRID_RESOLUTION)
    fov_mask = np.asarray(cam_masks["camera"], dtype=np.uint8) * 255
    return fov_mask

def calculate_fov(focal_length, image_width):
    # Calculate field of view in degrees
    fov_radians = 2 * np.arctan(image_width / (2 * focal_length))
    fov_degrees = np.degrees(fov_radians)
    return fov_degrees

def rasterize_to_bev(points, resolution=0.5, grid_size=25):
    # Rasterize 3D points to a BEV occupancy map
    bev_map = np.zeros((int(grid_size), int(grid_size)))
    grid_coords = np.floor(points[:, :2] / resolution).astype(np.int32) + int(grid_size // 2)
    valid_points = (grid_coords[:, 0] >= 0) & (grid_coords[:, 0] < bev_map.shape[0]) & \
                   (grid_coords[:, 1] >= 0) & (grid_coords[:, 1] < bev_map.shape[1])
    bev_map[grid_coords[valid_points, 1], grid_coords[valid_points, 0]] = 255
    bev_map = np.flip(bev_map, axis=0)
    return bev_map

# Data processing and batching
def get_timestamps_from_filenames(filenames, sorted_order=True):
    # Extract numeric timestamps from filenames
    timestamps_set = {
        int(filename.split(".")[0]) for filename in filenames if filename.split(".")[0].isdigit()
    }
    timestamps = list(timestamps_set)
    if sorted_order:
        timestamps = sorted(timestamps)
    return timestamps

def get_missing_timestamps(timestamps, data_dirs):
    # Identify timestamps missing from any of the specified data directories
    missing_timestamps = set()
    for data_dir in data_dirs:
        existing_filenames = get_all_filenames(os.path.join(SOURCE_DIR, data_dir), no_extension=True)
        existing_timestamps = get_timestamps_from_filenames(existing_filenames, sorted_order=False)
        existing_timestamps_set = set(existing_timestamps)
        for timestamp in timestamps:
            if timestamp not in existing_timestamps_set:
                missing_timestamps.add(timestamp)
    missing_timestamps = sorted(list(missing_timestamps))
    return missing_timestamps

def get_corrected_point_clouds(obstacles_point_cloud, ground_point_cloud, height_range=(0.3, 1.5)):
    # Remove points far from ground mesh and correct ground/obstacle separation
    def gen_mesh(pcd): 
        try:
            points = np.asarray(pcd.points)
        except:
            points = pcd
        tri = Delaunay(points[:, :2])
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(points)
        mesh.triangles = o3d.utility.Vector3iVector(tri.simplices)
        return mesh
    def mesh_to_cloud_signed_distances(o3d_mesh: o3d.t.geometry.TriangleMesh, cloud: o3d.t.geometry.PointCloud) -> np.ndarray:
        scene = o3d.t.geometry.RaycastingScene()
        _ = scene.add_triangles(o3d_mesh)
        sdf = scene.compute_signed_distance(cloud.point.positions)
        return sdf.numpy()
    def filter_points_far_from_mesh(pcd, distances, t1, t2):
        indices1 = np.where((distances > t1) & (distances <= t2))[0]
        indices2 = np.where(distances < t1)[0]
        objects = pcd.select_by_index(indices1)
        ground = pcd.select_by_index(indices2)
        return objects, ground
    def remove_points_far_from_mesh(pcd, mesh, height_range=(0.4, 2)):
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        tpcd = o3d.t.geometry.PointCloud.from_legacy(pcd)
        sdf = mesh_to_cloud_signed_distances(mesh_t, tpcd)
        sdf = np.abs(sdf)
        obstacles, ground = filter_points_far_from_mesh(pcd, sdf, *height_range)
        return obstacles, ground
    try:
        ground_mesh = gen_mesh(ground_point_cloud)
        obstacles_point_cloud, removed_points = remove_points_far_from_mesh(obstacles_point_cloud, ground_mesh, height_range)
        ground_point_cloud += removed_points
    except:
        pass
    return obstacles_point_cloud, ground_point_cloud

def get_visible_voxels_point_cloud(point_cloud, cameras):
    # Compute visible voxels from a point cloud given camera views
    def pcd_to_voxel_indices(points: np.ndarray) -> np.ndarray:
        indices = np.floor(points / GRID_RESOLUTION)
        unique = np.unique(indices, axis=0)
        return unique
    def voxel_indices_to_pcd(voxel_indices: np.ndarray) -> np.ndarray:
        return (voxel_indices + 0.5) * GRID_RESOLUTION
    voxel_size = GRID_RESOLUTION
    voxel_centroids = pcd_to_voxel_indices(np.asarray(point_cloud.points))
    voxel_centroids = voxel_indices_to_pcd(voxel_centroids)
    viewshed = ViewShed3D(voxel_centroids, voxel_size)
    visible_voxels = []
    for camera in cameras:
        camera_matrix = camera.get_projection_matrix()
        camera_image_width = camera.get_native_image_width()
        camera_image_height = camera.get_native_image_height()
        voxels = viewshed.compute_visible_voxels(camera_matrix, camera_image_width, camera_image_height, bounds=GRID_SIZE//2)
        if len(voxels) > 0:
            visible_voxels.append(voxels)
    visible_voxels_point_cloud = o3d.geometry.PointCloud()
    if len(visible_voxels) > 0:
        visible_voxels = np.array(np.concatenate(visible_voxels))
        visible_voxels_point_cloud.points = o3d.utility.Vector3dVector(visible_voxels)
    return visible_voxels_point_cloud

def process_data(timestamp):
    # Main per-frame processing function
    
    # Load LiDAR transform for current timestamp as a reference frame
    lidar_transform = np.load(os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.npy"))
    
    # Process depth and semantic camera outputs for current timestamp and generate semantic point cloud
    depth_cameras = []
    depth_point_cloud = o3d.geometry.PointCloud()
    for (SEM_DIR, DEPTH_DIR, CAM_DIR) in zip(SEMANTIC_CAM_DIRS, DEPTH_CAM_DIRS, CAM_DIRS):
        depth_camera_transform_path = os.path.join(SOURCE_DIR, DEPTH_DIR, f"{timestamp}.npy")
        depth_camera_transform = np.load(depth_camera_transform_path)
        depth_image_path = os.path.join(SOURCE_DIR, DEPTH_DIR, f"{timestamp}.png")
        depth_image = read_image(depth_image_path) 
        semantic_image_path = os.path.join(SOURCE_DIR, SEM_DIR, f"{timestamp}.png")
        semantic_image = read_image(semantic_image_path)
        depth_camera_intrinsics = get_intrinsics_dict(DEPTH_DIR)
        depth_camera_intrinsics_matrix = get_intrinsics_matrix(DEPTH_DIR)
        depth_camera_extrinsics_matrix = np.dot(np.linalg.inv(lidar_transform), depth_camera_transform)
        depth_camera = Camera(depth_camera_intrinsics_matrix, depth_camera_extrinsics_matrix, name=DEPTH_DIR)
        depth_cameras.append(depth_camera)         
        point_cloud = depth_image_to_semantic_point_cloud(depth_image, semantic_image, depth_camera_intrinsics, max_distance=MAX_POSTPROCESSING_DISTANCE)
        point_cloud.transform(depth_camera_transform)
        depth_point_cloud += point_cloud
    depth_point_cloud.transform(np.linalg.inv(lidar_transform))
    
    # Remove vehicle points from point cloud using bounding box
    chassis_bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.75, -1.5, -2.0), max_bound=(0.75, 1.5, 1.0))
    indices = chassis_bbox.get_point_indices_within_bounding_box(depth_point_cloud.points)
    depth_point_cloud = depth_point_cloud.select_by_index(indices, invert=True)

    # Continue processing point cloud to generate BEV maps and visibility masks
    ground_point_cloud = get_ground_from_semantic_point_cloud(depth_point_cloud)
    ground_point_cloud = ground_point_cloud.voxel_down_sample(GRID_RESOLUTION)
    obstacles_point_cloud = get_obstacles_from_semantic_point_cloud(depth_point_cloud)
    obstacles_point_cloud, ground_point_cloud = get_corrected_point_clouds(obstacles_point_cloud, ground_point_cloud)
    if DATA_PROCESING_MODE == "debug":
        save_point_cloud(os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.obstacles.ply"), obstacles_point_cloud)
        save_point_cloud(os.path.join(SOURCE_DIR, DEPTH_VISIBILITY_DIR, f"{timestamp}.ply"), depth_point_cloud)
    visible_voxel_point_cloud = get_visible_voxels_point_cloud(depth_point_cloud, depth_cameras)
    visible_voxel_point_cloud, _ = get_corrected_point_clouds(visible_voxel_point_cloud, ground_point_cloud, height_range=(-0.05, 1.5))
    if DATA_PROCESING_MODE == "debug":
        save_point_cloud(os.path.join(SOURCE_DIR, DEPTH_VISIBILITY_DIR, f"{timestamp}.visibility.ply"), visible_voxel_point_cloud)
    cumulative_visibility_mask = rasterize_to_bev(np.asarray(visible_voxel_point_cloud.points), resolution=GRID_RESOLUTION, grid_size=GRID_SIZE)
    save_image(os.path.join(SOURCE_DIR, DEPTH_VISIBILITY_DIR, f"{timestamp}.png"), cumulative_visibility_mask)
    cumulative_occupancy_image = rasterize_to_bev(np.asarray(obstacles_point_cloud.points), resolution=GRID_RESOLUTION, grid_size=GRID_SIZE)
    cumulative_occupancy_image[cumulative_visibility_mask == 0] = 0
    save_image(os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.bev.png"), cumulative_occupancy_image)
    def get_signed_distance_field_from_occupancy_image(occupancy_image):
        img = np.array(occupancy_image)
        inv_arr = (255 - np.array(img))
        sdf = distance_transform_edt(inv_arr).astype(np.float32)
        sdf += 1
        sdf = 1 / sdf 
        return sdf
    def get_sdf_as_image(sdf_array):
        sdf_array = (sdf_array / sdf_array.max() * 255).astype(np.uint8)
        sdf_image = cv2.applyColorMap(sdf_array, cv2.COLORMAP_JET)
        return sdf_image
    sdf = get_signed_distance_field_from_occupancy_image(cumulative_occupancy_image)
    np.save(os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.sdf.npy"), sdf)
    sdf_image = get_sdf_as_image(sdf)
    save_image(os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.sdf.png"), sdf_image)
    reference_frame_transform = lidar_transform
    np.save(os.path.join(SOURCE_DIR, DEPTH_VISIBILITY_DIR, f"{timestamp}.npy"), reference_frame_transform)
    np.save(os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.npy"), reference_frame_transform)
    for CAM_DIR, DEPTH_DIR in zip(CAM_DIRS, DEPTH_CAM_DIRS):
        depth_image_path = os.path.join(SOURCE_DIR, DEPTH_DIR, f"{timestamp}.png")
        depth_image = read_image(depth_image_path)
        depth_camera_transform_path = os.path.join(SOURCE_DIR, DEPTH_DIR, f"{timestamp}.npy")
        depth_camera_transform = np.load(depth_camera_transform_path)
        lidar_transform_path = os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.npy")
        lidar_transform = np.load(lidar_transform_path)
        lidar_transform_inv = np.linalg.inv(lidar_transform)
        combined_transform = np.dot(lidar_transform_inv, depth_camera_transform)
        intrinsics_matrix = get_intrinsics_matrix(DEPTH_DIR)
        camera_fov_mask = get_fov_mask(depth_image, combined_transform, intrinsics_matrix)
        fov_mask_path = os.path.join(SOURCE_DIR, DEPTH_DIR, f"{timestamp}.fov.png")
        save_image(fov_mask_path, camera_fov_mask)
        camera_visibility_mask = np.array((cumulative_visibility_mask > 0) & (camera_fov_mask > 0), dtype=np.uint8) * 255
        visibility_mask_path = os.path.join(SOURCE_DIR, DEPTH_DIR, f"{timestamp}.visibility.png")
        save_image(visibility_mask_path, camera_visibility_mask)
    def get_lidar_obstacle_point_cloud(semantic_point_cloud):
        # Extract obstacle points from LIDAR semantic point cloud
        semantic_colors = np.asarray(semantic_point_cloud.colors)
        r_channel = semantic_colors[:, 0] * 255
        mask = np.isin(
            r_channel, 
            [ 
                SemanticTags.ROADLINE.value, SemanticTags.ROAD.value, SemanticTags.SIDEWALK.value,
                SemanticTags.GROUND.value, SemanticTags.WATER.value, SemanticTags.TERRAIN.value, 
                SemanticTags.SKY.value
            ],
            invert=True
        )
        filtered_points = np.asarray(semantic_point_cloud.points)[mask]
        filtered_colors = semantic_colors[mask]
        filtered_point_cloud = o3d.geometry.PointCloud()
        filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
        return filtered_point_cloud
    def get_lidar_ground_point_cloud(semantic_point_cloud):
        # Extract ground points from LIDAR semantic point cloud
        semantic_colors = np.asarray(semantic_point_cloud.colors)
        r_channel = semantic_colors[:, 0] * 255
        mask = np.isin(
            r_channel, 
            [
                SemanticTags.ROADLINE.value, SemanticTags.ROAD.value, 
                SemanticTags.SIDEWALK.value, SemanticTags.GROUND.value, 
                SemanticTags.WATER.value, SemanticTags.TERRAIN.value,
                SemanticTags.UNLABELED.value
            ]
        )
        filtered_points = np.asarray(semantic_point_cloud.points)[mask]
        filtered_colors = semantic_colors[mask]
        filtered_point_cloud = o3d.geometry.PointCloud()
        filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_point_cloud.colors = o3d.utility.Vector3dVector(filtered_colors)
        return filtered_point_cloud
    if (DATA_PROCESING_MODE == "full") or (DATA_PROCESING_MODE == "debug"):
        lidar_point_cloud_path = os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.ply")
        lidar_point_cloud = o3d.io.read_point_cloud(lidar_point_cloud_path)
        lidar_ground_point_cloud = get_lidar_ground_point_cloud(lidar_point_cloud)
        lidar_obstacle_point_cloud = get_lidar_obstacle_point_cloud(lidar_point_cloud)
        lidar_obstacle_point_cloud, lidar_ground_point_cloud = get_corrected_point_clouds(lidar_obstacle_point_cloud, lidar_ground_point_cloud)
        if DATA_PROCESING_MODE == "debug":
            save_point_cloud(os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.obstacles.ply"), lidar_obstacle_point_cloud)
            save_point_cloud(os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.ground.ply"), lidar_ground_point_cloud)
        lidar_bev_image = rasterize_to_bev(np.asarray(lidar_obstacle_point_cloud.points), resolution=GRID_RESOLUTION, grid_size=GRID_SIZE)
        lidar_bev_image_path = os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.bev.png")
        save_image(lidar_bev_image_path, lidar_bev_image)

def split_array_to_batches(array, batch_size):
    # Split array into batches of specified size
    return [array[i:i + batch_size] for i in range(0, len(array), batch_size)]

if CLEAN_SOURCE_DIR:
    clean_up_lidar_dir()
    clean_up_depth_camera_dirs()
    clean_up_depth_bev_dir()
    clean_up_depth_visibility_dir()

lidar_filenames = get_all_filenames(os.path.join(SOURCE_DIR, LIDAR_DIR))
lidar_timestamps = get_timestamps_from_filenames(lidar_filenames)
missing_timestamps = get_missing_timestamps(
    lidar_timestamps, 
    [os.path.join(SOURCE_DIR, sensor_dir) for sensor_dir in CAM_DIRS + DEPTH_CAM_DIRS + SEMANTIC_CAM_DIRS]
)
if len(missing_timestamps) > 0:
    print(f"Warning: The following timestamps are missing data and will be skipped during processing: {sorted(list(missing_timestamps))}")
timestamps = [timestamp for timestamp in lidar_timestamps if timestamp not in missing_timestamps]

if START_FROM_TIMESTAMP is not None:
    frame_index = timestamps.index(START_FROM_TIMESTAMP)
    timestamps = timestamps[frame_index:]
if END_AT_TIMESTAMP is not None:
    frame_index = timestamps.index(END_AT_TIMESTAMP) + 1
    timestamps = timestamps[:frame_index]
timestamp_batches = split_array_to_batches(timestamps, BATCH_SIZE)

print(f"Post-processing simulation data ...")
for i, timestamp_batch in enumerate(timestamp_batches):
    Parallel(n_jobs=N_WORKERS)(delayed(process_data)(timestamp) for timestamp in timestamp_batch)
    print(f"{datetime.now()} Processed {((i+1) / len(timestamp_batches) * 100):.6f}% of frames in the dataset. ({timestamps.index(timestamp_batch[-1]) + 1} out of {len(timestamps)} frames)")
print(f"Processed data in {SOURCE_DIR}")

# Export processed files to target directory in ML pipeline format
TARGET_CAM_DIRS = ["CAM_FRONT", "CAM_FRONT_LEFT", "CAM_FRONT_RIGHT", "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT"]
TARGET_LIDAR_DIR = "LIDAR_TOP"
TARGET_FOV_MASKS_DIR = "fov_masks"
TARGET_BEVS_DIR = "occ"
TARGET_SDFS_DIR = "soft_occ"
TARGET_VISIBILITY_MASKS_DIR = "visibility_masks"
TARGET_CUMULATIVE_MASKS_DIR = "cumulative_masks"

def clean_up_target_dir():
    # Remove all subdirectories from target directory
    if not os.path.exists(TARGET_DIR):
        return
    for subdir in os.listdir(TARGET_DIR):
        shutil.rmtree(os.path.join(TARGET_DIR, subdir))

def copy_file_to_target_dir(source_file_path, target_file_path):
    # Copy file to target directory, creating directories as needed
    target_directory_path = os.path.dirname(target_file_path)
    os.makedirs(target_directory_path, exist_ok=True)
    shutil.copyfile(source_file_path, target_file_path)

clean_up_target_dir()    

print(f"Exporting post-processed data...")
for i, timestamp in enumerate(timestamps):
    if (i % N_FRAMES_PER_BAG) == 0:
        start_timestamp = timestamps[i]
        end_timestamp = timestamps[-1] if (i+N_FRAMES_PER_BAG-1) >= len(timestamps) else timestamps[i+N_FRAMES_PER_BAG-1]
        TARGET_DIRNAME = os.path.basename(TARGET_DIR)
        TARGET_BAG_DIR = f"{TARGET_DIRNAME}_frames_{start_timestamp}_{end_timestamp}"
    for CAM_DIR in CAM_DIRS:
        source_file_path = os.path.join(SOURCE_DIR, CAM_DIR, f"{timestamp}.png")
        target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, CAM_DIR, f"{timestamp}.png")
        copy_file_to_target_dir(source_file_path, target_file_path)
        source_file_path = os.path.join(SOURCE_DIR, CAM_DIR, f"{timestamp}.npy")
        target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, CAM_DIR, f"{timestamp}.npy")
        copy_file_to_target_dir(source_file_path, target_file_path)
    source_file_path = os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.ply")
    target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, LIDAR_DIR, f"{timestamp}.ply")
    copy_file_to_target_dir(source_file_path, target_file_path)
    source_file_path = os.path.join(SOURCE_DIR, LIDAR_DIR, f"{timestamp}.npy")
    target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, LIDAR_DIR, f"{timestamp}.npy")
    copy_file_to_target_dir(source_file_path, target_file_path)
    for CAM_DIR, DEPTH_CAM_DIR in zip(CAM_DIRS, DEPTH_CAM_DIRS):
        source_file_path = os.path.join(SOURCE_DIR, DEPTH_CAM_DIR, f"{timestamp}.fov.png")
        target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_FOV_MASKS_DIR, CAM_DIR, f"{timestamp}.png")
        copy_file_to_target_dir(source_file_path, target_file_path)
        source_file_path = os.path.join(SOURCE_DIR, DEPTH_CAM_DIR, f"{timestamp}.npy")
        target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_FOV_MASKS_DIR, CAM_DIR, f"{timestamp}.npy")
        copy_file_to_target_dir(source_file_path, target_file_path)
        source_file_path = os.path.join(SOURCE_DIR, DEPTH_CAM_DIR, f"{timestamp}.visibility.png")
        target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_VISIBILITY_MASKS_DIR, CAM_DIR, f"{timestamp}.png")
        copy_file_to_target_dir(source_file_path, target_file_path)
        source_file_path = os.path.join(SOURCE_DIR, DEPTH_CAM_DIR, f"{timestamp}.npy")
        target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_VISIBILITY_MASKS_DIR, CAM_DIR, f"{timestamp}.npy")
        copy_file_to_target_dir(source_file_path, target_file_path)
    source_file_path = os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.bev.png")
    target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_BEVS_DIR, f"{timestamp}.png")
    copy_file_to_target_dir(source_file_path, target_file_path)
    source_file_path = os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.sdf.npy")
    target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_SDFS_DIR, f"{timestamp}.npy")
    copy_file_to_target_dir(source_file_path, target_file_path)
    source_file_path = os.path.join(SOURCE_DIR, DEPTH_BEV_DIR, f"{timestamp}.sdf.png")
    target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_SDFS_DIR, f"{timestamp}.png")
    copy_file_to_target_dir(source_file_path, target_file_path)
    source_file_path = os.path.join(SOURCE_DIR, DEPTH_VISIBILITY_DIR, f"{timestamp}.png")
    target_file_path = os.path.join(TARGET_DIR, TARGET_BAG_DIR, TARGET_VISIBILITY_MASKS_DIR, TARGET_CUMULATIVE_MASKS_DIR, f"{timestamp}.png")
    copy_file_to_target_dir(source_file_path, target_file_path)
    if i % 10 == 0:
        print(f"{datetime.now()} Exported {((i+1) / len(timestamps) * 100):.6f}% of frames in the dataset. ({i} out of {len(timestamps)} frames)")
print(f"Exported {((i+1) / len(timestamps) * 100):.6f}% of frames in the dataset")
print(f"Exported post-processed data to {TARGET_DIR}")

print(f"Post-processing completed!")
