import carla
import time
import argparse
import os
import numpy as np
import cv2
import json
import urdf_parser_py.urdf as urdf
from scipy.spatial.transform import Rotation
import datetime
import open3d as o3d
import sys
import re


# GLOBAL VARIABLES
client = carla.Client('localhost', 2000)
client.set_timeout(100)
world = client.get_world()
map = world.get_map()
blueprint_library = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
# END OF GLOBAL VARIABLES

class URDFParser:
    def __init__(self, urdf_file):
        self.urdf_file = urdf_file
        self.robot = urdf.URDF.from_xml_file(urdf_file)
        self.root = self.robot.get_root()

    def compute_chain_transform(self, chain):
        transform = np.eye(4)
        
        for joint in chain:
            if joint not in self.robot.joint_map:
                continue
            
            joint_info = self.robot.joint_map[joint]
            rpy = joint_info.origin.rpy
            xyz = joint_info.origin.xyz
            rotation = Rotation.from_euler('xyz', rpy).as_matrix()
            translation = np.array(xyz)
            T = self.build_transform_matrix(rotation, translation)
            transform = np.dot(transform, T)
        
        return transform

    def get_T_from_to(self, start_frame, end_frame):
        chain_1 = self.robot.get_chain(self.root, start_frame)
        chain_2 = self.robot.get_chain(self.root, end_frame)
        T1 = self.compute_chain_transform(chain_1)
        T2 = self.compute_chain_transform(chain_2)
        return np.dot(np.linalg.inv(T1), T2)
    
    def build_transform_matrix(self, rotation, translation):
        m = np.eye(4)
        m[:3, :3] = rotation
        m[:3, 3] = translation
        return m

def get_recording_info(client, rec_file):
    """Extract map name and frames from a CARLA recording file."""
    info = client.show_recorder_file_info(rec_file, False)
    # Write info as log file to recording_info.log
    with open("recording_info.log", "w") as f:
        f.write(info)
    map_name = None
    frames = None
    recording_duration = None
    for line in info.split('\n'):
        if 'Map:' in line:
            map_name = line.split('Map:')[1].strip()
        elif 'Frames:' in line:
            frames = line.split('Frames:')[1].strip()
        elif 'Duration:' in line:
            duration_str = line.split('Duration:')[1].strip()
            # Match duration in seconds format (e.g., 15.5674 seconds)
            match = re.match(r'([\d\.]+) seconds', duration_str)
            recording_duration = float(match.group(1))
    if map_name is None or frames is None:
        raise ValueError("Could not extract all recording info from the file.")
    return map_name, frames, recording_duration
    
def matrix_to_transform(matrix):
    """Convert a 4x4 transformation matrix to a CARLA Transform."""
    if not isinstance(matrix, np.ndarray):
        matrix = np.array(matrix)
    location = carla.Location(x=matrix[0, 3], y=(-matrix[1, 3]), z=matrix[2, 3])
    roll, pitch, yaw = Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz', degrees=True)
    rotation = carla.Rotation(pitch=(-pitch), yaw=(-yaw), roll=roll)
    return carla.Transform(location, rotation)
    
def rotation_matrix(axis, angle):
    """Return a 4x4 rotation matrix for a given axis and angle."""
    if axis == 'x':
        return np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'y':
        return np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'z':
        return np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
def transform_to_carla(sensor_type, transformation):
    """Convert a sensor transformation to CARLA's coordinate system."""
    sensor_type = sensor_type.lower().strip()
    if sensor_type in ['camera', 'sensor.camera', 'sensor.camera.rgb', 'sensor.camera.semantic_segmentation', 'sensor.camera.instance_segmentation', 'sensor.camera.depth']:
        rotation1 = rotation_matrix('z', np.pi / 2)
        rotation2 = rotation_matrix('y', -np.pi / 2)
        rotation = np.dot(rotation1, rotation2)
    elif sensor_type in ['lidar', 'sensor.lidar', 'sensor.lidar.ray_cast', 'sensor.lidar.ray_cast_semantic']:
        rotation = rotation_matrix('z', np.pi / 2)
    elif sensor_type in ['radar', 'sensor.other.radar']:
        rotation = np.eye(4)
    else:
        raise ValueError(f"Unknown sensor type: {sensor_type}")
    tf = np.dot(transformation, rotation)
    return tf

def transform_from_carla(sensor_type):
    """Get the inverse transformation from CARLA's coordinate system to sensor."""
    sensor_type = sensor_type.lower().strip()
    original_tf = transform_to_carla(sensor_type, np.eye(4))
    inverse_tf = np.linalg.inv(original_tf)
    return inverse_tf

def convert_point_cloud_from_left_to_right_hand_system(point_cloud):
    """Convert point cloud from CARLA (left-handed) to right-handed system."""
    flip_yz_matrix = np.array([
                        [-1, 0,  0, 0],
                        [ 0, 1,  0, 0],
                        [ 0, 0,  1, 0],
                        [ 0, 0,  0, 1]
                    ])
    rotation = rotation_matrix('z', np.pi / 2)
    tf_to_SE3 = flip_yz_matrix @ rotation
    point_cloud.transform(tf_to_SE3)
    return point_cloud

def save_sensor_position(raw_data, target_directory, sensor_type=None):
    """Save the 4x4 transformation matrix of a sensor to a .npy file."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    transform_matrix = raw_data.transform.get_matrix()
    if "camera" in sensor_type:
        rotation_inv = transform_from_carla(sensor_type)
    else:
        rotation = rotation_matrix('z', np.pi / 2)
        rotation_inv = np.linalg.inv(rotation)
    flip_xz_matrix = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0,  1, 0],
        [0,  0,  0, 1]
    ])
    flip_yz_matrix = np.array([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 0],
        [ 0, 0,  1, 0],
        [ 0, 0,  0, 1]
    ])
    transform_matrix = flip_xz_matrix @ transform_matrix @ rotation_inv @ flip_yz_matrix
    filename =f"{int(raw_data.timestamp*1000)}.npy"
    filepath = os.path.join(target_directory, filename)
    np.save(filepath, transform_matrix)

def save_sensor_position(raw_data, target_directory, sensor_type=None):
    """Save the 4x4 transformation matrix of a sensor to a .npy file."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    transform_matrix = raw_data.transform.get_matrix()
    if "camera" in sensor_type:
        rotation_inv = transform_from_carla(sensor_type)
    else:
        rotation = rotation_matrix('z', np.pi / 2)
        rotation_inv = np.linalg.inv(rotation)
    flip_xz_matrix = np.array([
        [1,  0,  0, 0],
        [0, -1,  0, 0],
        [0,  0,  1, 0],
        [0,  0,  0, 1]
    ])
    flip_yz_matrix = np.array([
        [-1, 0,  0, 0],
        [ 0, 1,  0, 0],
        [ 0, 0,  1, 0],
        [ 0, 0,  0, 1]
    ])
    transform_matrix = flip_xz_matrix @ transform_matrix @ rotation_inv @ flip_yz_matrix
    filename = f"{int(raw_data.timestamp*1000)}.npy"
    filepath = os.path.join(target_directory, filename)
    np.save(filepath, transform_matrix)

def save_lidar_readings(raw_data, target_directory):
    """Save lidar point cloud as .ply file."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    points = np.array([(detection.point.x, detection.point.y, detection.point.z) for detection in raw_data])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud = convert_point_cloud_from_left_to_right_hand_system(point_cloud)
    filename = f"{int(raw_data.timestamp*1000)}.ply"
    filepath = os.path.join(target_directory, filename)
    o3d.io.write_point_cloud(filepath, point_cloud)

def spawn_single_sensor(vehicle, ego_vehicle_extrinsics, ego_vehicle_intrinsics, sensor_output_path):
    """Spawn only a specific sensor"""
    def listener_factory(sensor_type, output_dir):
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        def save_image_to_disk(image):
            # Ignore CollistionEvent 
            if hasattr(image, 'type') and image.type == 'CollisionEvent':
                print("Received CollisionEvent, skipping...")
                return
            timestamp = float(image.timestamp)
            timestamp_ms = timestamp * 1000
            timestamp = f"{int(timestamp_ms)}"
            image_path = f"{output_dir}/{timestamp}.png"
            image.save_to_disk(image_path)
            save_sensor_position(image, output_dir, sensor_type="sensor.camera.rgb")
        def save_pcd_to_disk(point_cloud):
            if hasattr(point_cloud, 'type') and point_cloud.type == 'CollisionEvent':
                print("Received CollisionEvent, skipping...")
                return
            save_lidar_readings(point_cloud, output_dir)
            save_sensor_position(point_cloud, output_dir, sensor_type="sensor.lidar.ray_cast_semantic")
        if ".lidar." in sensor_type:
            return save_pcd_to_disk
        elif ".camera." in sensor_type:
            return save_image_to_disk
        
        print(f"Warning: No listener defined for sensor {sensor_type}, using dummy listener.")
        return lambda x: None

    target_sensor_name = sensor_output_path.split("/")[-1]
    print(f"Spawning sensor: {target_sensor_name}")
    sensors = []
    
    if "CAM_" in target_sensor_name:
        # RGB Camera
        sensor_type = "sensor.camera.rgb"
        blueprint = blueprint_library.find(sensor_type)
        sensor_intrinsics = ego_vehicle_intrinsics.get(target_sensor_name, dict())
        
        def calculate_fov(focal_length, image_width):
            fov_radians = 2 * np.arctan(image_width / (2 * focal_length))
            return np.degrees(fov_radians)

        image_width = str(sensor_intrinsics.get("w", 1600))
        blueprint.set_attribute('image_size_x', image_width)
        image_height = str(sensor_intrinsics.get("h", 900))
        blueprint.set_attribute('image_size_y', image_height)
        focal_distance = float(sensor_intrinsics.get("fl", 800))
        field_of_view = str(calculate_fov(focal_distance, float(image_width)))
        blueprint.set_attribute('fov', field_of_view)
        transform_matrix = ego_vehicle_extrinsics.get_T_from_to("base_link", target_sensor_name)
        transform_matrix = transform_to_carla("sensor.camera.rgb", transform_matrix)
        transform = matrix_to_transform(transform_matrix)
        
        sensor_rgb = world.spawn_actor(blueprint, transform, attach_to=vehicle)
        sensor_rgb.listen(listener_factory(sensor_type, sensor_output_path))
        sensors.append(sensor_rgb)
        print(f"Spawned RGB camera {target_sensor_name}")

        # Spawn Depth Camera in same location
        sensor_type = "sensor.camera.depth"
        blueprint = blueprint_library.find(sensor_type)
        blueprint.set_attribute('image_size_x', image_width)
        blueprint.set_attribute('image_size_y', image_height)
        blueprint.set_attribute('fov', field_of_view)
        sensor_depth = world.spawn_actor(blueprint, transform, attach_to=vehicle)
        sensor_depth.listen(listener_factory(sensor_type, sensor_output_path.replace("CAM_", "DEPTH_CAM_")))
        sensors.append(sensor_depth)
        print(f"Spawned Depth camera DEPTH_{target_sensor_name}")
        # Spawn Semantic Camera in same location
        sensor_type = "sensor.camera.instance_segmentation"
        blueprint = blueprint_library.find(sensor_type)
        blueprint.set_attribute('image_size_x', image_width)
        blueprint.set_attribute('image_size_y', image_height)
        blueprint.set_attribute('fov', field_of_view)
        sensor_sem = world.spawn_actor(blueprint, transform, attach_to=vehicle)
        sensor_sem.listen(listener_factory(sensor_type, sensor_output_path.replace("CAM_", "SEMANTIC_CAM_")))
        sensors.append(sensor_sem)
        print(f"Spawned Semantic camera SEMANTIC_{target_sensor_name}")

    elif "LIDAR_" in target_sensor_name:
        # Spawn semantic lidar
        sensor_type = "sensor.lidar.ray_cast_semantic"
        blueprint = blueprint_library.find(sensor_type)
        blueprint.set_attribute("channels", str(64))
        blueprint.set_attribute("points_per_second", str(100000))
        blueprint.set_attribute("range", str(100))
        blueprint.set_attribute("rotation_frequency", str(10))
        blueprint.set_attribute("upper_fov", str(15))
        blueprint.set_attribute("lower_fov", str(-15))
        
        transform_matrix = ego_vehicle_extrinsics.get_T_from_to("base_link", target_sensor_name)
        transform_matrix = transform_to_carla("sensor.lidar.ray_cast", transform_matrix)
        transform = matrix_to_transform(transform_matrix)
        
        sensor = world.spawn_actor(blueprint, transform, attach_to=vehicle)
        sensor.listen(listener_factory(sensor_type, sensor_output_path))
        sensors.append(sensor)
        print(f"Spawned LIDAR sensor {target_sensor_name}")

    return sensors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording', type=str, required=True, help='Path to the simulation recording (.rec) file')
    parser.add_argument('--ego_vehicle_extrinsics', type=str, required=True, help='Path to the URDF file for the vehicle')
    parser.add_argument('--ego_vehicle_intrinsics', type=str, required=True, help='Path to the JSON file with camera intrinsics')
    parser.add_argument('--output_dir', type=str, required=True, help='Directory to save the sensor data')
    args = parser.parse_args()

    recording_path = str(args.recording)
    if not os.path.exists(recording_path):
        print(f"Error: Recording file {recording_path} does not exist.")
        return 1
    extrinsics_path = args.ego_vehicle_extrinsics
    if not os.path.exists(extrinsics_path):
        print(f"Error: URDF file {extrinsics_path} does not exist.")
        return 1
    intrinsics_path = args.ego_vehicle_intrinsics
    if not os.path.exists(intrinsics_path):
        print(f"Error: Intrinsics file {intrinsics_path} does not exist.")
        return 1
    
    print(f"Replaying recording: {recording_path}")
    print(f"Output directory: {args.output_dir}")
    client = carla.Client('localhost', 2000)
    client.set_timeout(30.0)
    map_name, frames, recording_duration = get_recording_info(client, recording_path)
    frames = int(recording_duration / 0.1) # Assuming 10 FPS for replay
    # print(client.show_recorder_file_info(recording_path))

    world = client.load_world(map_name)
    settings = world.get_settings()
    settings.synchronous_mode = True  # Synchronous
    settings.fixed_delta_seconds = 0.1  # Adjust as needed for your simulation speed
    world.apply_settings(settings)

    vehicle_sensor_extrinsics = URDFParser(extrinsics_path)
    # Get all sensor configurations
    sensor_configs = []
    for sensor_configuration in vehicle_sensor_extrinsics.robot.links:
        sensor_name = sensor_configuration.name
        if "CAM_" in sensor_name or "LIDAR_" in sensor_name:
            sensor_configs.append(sensor_name)
    # Set CAM_FRONT as the first sensor to spawn
    if "CAM_FRONT" in sensor_configs:
        sensor_configs.remove("CAM_FRONT")
        sensor_configs.insert(0, "CAM_FRONT")

    
    for sensor_name in sensor_configs:
        client.load_world(map_name, reset_settings=False) # client.load_world(map_name, reset_settings=False) # Reload the world to ensure it is in a clean state
        for i in range(3000):
            world.tick()  # Ensure the world is updated before proceeding
            # time.sleep(0.1)  # Wait for the world to stabilize

        # Replay the recording (start paused)
        client.replay_file(recording_path, 0, 0, 0, True)
        client.set_replayer_time_factor(1.0)  # Pause the replay immediately
        world.tick()  # Ensure the world is updated before proceeding

        # Find ego vehicle (assumes one spectator-controlled vehicle)
        def get_hero_actor(world):
            actors = world.get_actors()
            hero_actors = []
            for actor in actors:
                if actor.attributes.get('role_name') == 'hero':
                    hero_actors.append(actor)
            print(f"Found {len(hero_actors)} hero actors out of {len(actors)} actors.")
            print("Hero actors:", [actor.id for actor in hero_actors])
            return hero_actors[0]
        
        vehicle = get_hero_actor(world)
        print("actor id:", vehicle.id, "type:", vehicle.type_id, "role:", vehicle.attributes.get('role_name'))

        vehicle_sensor_intrinsics = {}
        with open(intrinsics_path, 'r') as f:
            vehicle_sensor_intrinsics = json.load(f)
        
        sensors = spawn_single_sensor(vehicle, vehicle_sensor_extrinsics, vehicle_sensor_intrinsics, os.path.join(args.output_dir, sensor_name))
        print(f"Simulating sensor {sensor_name} in replay mode...")

        try:
            # number_of_ticks = 3331  # Simulate for 300 ticks per sensor
            number_of_ticks = int(frames) # 7580 #700  
            for tick_count in range(number_of_ticks):  # Simulate for 300 ticks per sensor
                world.tick()
                time.sleep(0.5)  # Adjust sleep time as needed for your simulation speed
                if tick_count % 100 == 0:
                    # Print tick count out of total with timestamp
                    print(f"{datetime.datetime.now()} Tick {tick_count}/{number_of_ticks} for sensor {sensor_name}")
        except KeyboardInterrupt:
            print("Replay stopped by user.")
        finally:
            print("Replay stopped.")

        for sensor in sensors:
            sensor.stop()
            sensor.destroy()
        vehicle.destroy()
        
    
    print("Exiting simulation.")
        
        

if __name__ == '__main__':
    main()
