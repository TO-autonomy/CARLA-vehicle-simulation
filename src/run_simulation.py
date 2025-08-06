import json
import math
import random
import time
import queue
import shutil 
import os
import sys
from enum import Enum
from datetime import datetime

import carla
import numpy as np
import urdf_parser_py.urdf as urdf
from scipy.spatial.transform import Rotation   
import matplotlib.pyplot as plt
from matplotlib import cm
import open3d as o3d
import cv2
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import argparse

# Set the CARLA Python API path
CURRENT_WORKING_DIR = os.getcwd()
CARLA_PYTHON_API_PATH = os.path.join(CURRENT_WORKING_DIR, "CARLASimulator", "PythonAPI", "carla")
if CARLA_PYTHON_API_PATH not in sys.path:
    sys.path.append(CARLA_PYTHON_API_PATH)

# Import CARLA navigation agents
try:
    from agents.navigation.basic_agent import BasicAgent
    from agents.navigation.behavior_agent import BehaviorAgent
except ImportError as error:
    raise ImportError(f"FATAL ERROR: Unable to import CARLA autonomous driving agent BasicAgent due to missing PythonAPI. The API is included in the simulator installation package (not included with 'import carla'). Setup the CARLA simulator repository and add the correct PythonAPI path above. ({error})")

# --- Command-line arguments and configuration loading ---

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

# Parse arguments (use defaults if running in Jupyter)
if 'ipykernel_launcher.py' in sys.argv[0]: 
    args = argparse.Namespace(
        ego_vehicle_extrinsics='/home/leppsalu/Desktop/Github/voxel-visibility-multithreaded/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf',
        ego_vehicle_intrinsics='/home/leppsalu/Desktop/Github/voxel-visibility-multithreaded/CARLA-vehicle-simulation/src/config/carla_intrinsics.json',
        episode_config='/home/leppsalu/Desktop/Github/voxel-visibility-multithreaded/CARLA-vehicle-simulation/src/config/town03.path.json',
        output_dir='/media/leppsalu/SSD_Storage/generated_data_town03_sample',
        skip_validation=True,
        toggle_off_buildings=False,
        n_pedestrians=15,
        n_vehicles=20
    )
else:
    parser = argparse.ArgumentParser(description='Run simulation postprocessing.')
    parser.add_argument('--ego_vehicle_extrinsics', type=str, required=False, default='/home/leppsalu/Desktop/Github/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf')
    parser.add_argument('--ego_vehicle_intrinsics', type=str, required=False, default='/home/leppsalu/Desktop/Github/CARLA-vehicle-simulation/src/config/carla_intrinsics.json')
    parser.add_argument('--episode_config', type=str, required=False, default='/home/leppsalu/Desktop/Github/CARLA-vehicle-simulation/src/config/town02.path.json')
    parser.add_argument('--output_dir', type=str, required=False, default='/home/leppsalu/Desktop/Github/CARLA-vehicle-simulation/src/generated_data')
    parser.add_argument('--skip_validation', action='store_true')
    parser.add_argument('--toggle_off_buildings', action='store_true')
    parser.add_argument('--n_pedestrians', type=int, required=False, default=0)
    parser.add_argument('--n_vehicles', type=int, required=False, default=0)
    args = parser.parse_args()

EGO_VEHICLE_EXTRINSICS = args.ego_vehicle_extrinsics
ego_vehicle_extrinsics = URDFParser(EGO_VEHICLE_EXTRINSICS)

EGO_VEHICLE_INTRINSICS = args.ego_vehicle_intrinsics
with open(EGO_VEHICLE_INTRINSICS) as intrinsics_file:
    ego_vehicle_intrinsics = json.load(intrinsics_file)

EPISODE_CONFIG_PATH = args.episode_config
with open(EPISODE_CONFIG_PATH) as path_file:
    episode_config_json = json.load(path_file)
episode_map = episode_config_json["map"]
episode_ego_vehicle_path = episode_config_json["route"]

SIMULATION_DATA_OUTPUT_PATH = args.output_dir
SKIP_VALIDATION = args.skip_validation
TOGGLE_OFF_BUILDINGS = args.toggle_off_buildings
N_PEDESTRIANS = args.n_pedestrians
N_VEHICLES = args.n_vehicles

# --- CARLA server connection and world setup ---

client = carla.Client('localhost', 2000)
client.set_timeout(100)
world = client.get_world()
map = world.get_map()
blueprint_library = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

def reload_world():
    global world, map, blueprint_library, spawn_points, traffic_manager
    world = client.reload_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    spawn_points = map.get_spawn_points()
    traffic_manager = client.get_trafficmanager()

def load_world(map_name="Town01", timeout=10.0):
    global world, map, blueprint_library, spawn_points, traffic_manager
    client.set_timeout(timeout)
    world = client.load_world(map_name)
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    spawn_points = map.get_spawn_points()
    traffic_manager = client.get_trafficmanager()

load_world(map_name=episode_map)
time.sleep(20)
# reload_world()

# --- Synchronous simulation mode context manager ---

class CarlaSyncMode(object):
    """
    Context manager for running CARLA in synchronous mode with sensors.
    Usage:
        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)
    """
    def __init__(self, world, sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 10)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds,
        ))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout=300):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

# --- Transformation utilities for CARLA and sensors ---

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

def reflection_matrix():
    """Return a 4x4 reflection matrix to flip the Y-axis."""
    return np.array([
        [-1, 0, 0, 0],
        [0, 1, 0, 0],
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

# --- Path validation and visualization ---

def get_agent_path(coordinates):
    """Convert a list of (x, y, z) coordinates to CARLA waypoints."""
    path = []
    for x, y, z in coordinates:
        location = carla.Location(x=x, y=y, z=z)
        waypoint = map.get_waypoint(
            location,
            project_to_road=True
            )
        waypoint_location = waypoint.transform.location
        path.append(waypoint_location)
    return path

def is_reachable(start_wp, target_wp, max_depth=500, step=2.0):
    """Check if target waypoint is reachable from start waypoint."""
    current_wps = [start_wp]
    visited = set()

    for _ in range(max_depth):
        next_wps = []
        for wp in current_wps:
            if wp.road_id == target_wp.road_id and wp.lane_id == target_wp.lane_id:
                if abs(wp.s - target_wp.s) < step:
                    return True
            visited.add((wp.road_id, wp.lane_id, round(wp.s, 1)))
            for nwp in wp.next(step):
                node = (nwp.road_id, nwp.lane_id, round(nwp.s, 1))
                if node not in visited:
                    next_wps.append(nwp)
        current_wps = next_wps
        if not current_wps:
            break
    return False


def find_nearest_spawn_point(location):
    """Find the nearest REACHABLE spawn point to a given location."""
    global spawn_points, map

    nearest_spawn_point = None
    min_distance = float('inf')
    target_wp = map.get_waypoint(location, project_to_road=True)

    for spawn_point in spawn_points:
        start_wp = map.get_waypoint(spawn_point.location, project_to_road=True)
        if not is_reachable(start_wp, target_wp):
            continue  # Skip unreachable points
        distance = spawn_point.location.distance(location)
        if distance < min_distance:
            min_distance = distance
            nearest_spawn_point = spawn_point

    return nearest_spawn_point

def validate_path():
    """Run a validation drive along the planned path and visualize with a camera."""
    # reload_world()
    cv2.namedWindow("Press Q to stop simulation")
    cv2.imshow("Press Q to stop simulation", np.zeros((1,1)))

    agent_path = get_agent_path(episode_ego_vehicle_path)

    blueprint_name = "vehicle.dodge.charger_2020"
    blueprint = blueprint_library.find(blueprint_name)
    transform = find_nearest_spawn_point(agent_path[0])
    validation_vehicle = world.spawn_actor(blueprint, transform)

    blueprint = blueprint_library.find("sensor.camera.rgb")
    blueprint.set_attribute('image_size_x', str(720))
    blueprint.set_attribute('image_size_y', str(480))

    transform_matrix = carla.Transform(carla.Location(x=1.7, y=0.0, z=1.5), carla.Rotation(roll=0, pitch=0, yaw=0)).get_matrix()
    transform = matrix_to_transform(transform_matrix)
    sensor = world.spawn_actor(blueprint, transform, attach_to=validation_vehicle)
    validation_vehicle_control_agent = BasicAgent(validation_vehicle)
    validation_vehicle_control_agent.ignore_traffic_lights()
    validation_vehicle_control_agent.ignore_stop_signs()
    validation_vehicle_control_agent.set_destination(agent_path.pop(0))

    agent_path_completed = False
    try:
        with CarlaSyncMode(world, [sensor], fps=10) as sync_mode:
            print("Driving to the start of the path...")
            while validation_vehicle_control_agent.done() is False:
                if (cv2.waitKey(1) == ord('q')):
                    break
                simulated_results = sync_mode.tick()[1:]
                cam_sensor = simulated_results[0]
                cam_sensor_image =  np.reshape(np.copy(cam_sensor.raw_data), (cam_sensor.height, cam_sensor.width, 4))
                cv2.imshow("CAM_FRONT", cam_sensor_image)
                validation_vehicle.apply_control(validation_vehicle_control_agent.run_step())
            print("Arrived at the start of the path!")

            print("Driving on path...")
            actual_path = []
            next_wp = None
            while True:
                if (cv2.waitKey(1) == ord('q')):
                    break
                simulated_results = sync_mode.tick()[1:]
                cam_sensor = simulated_results[0]
                cam_sensor_image =  np.reshape(np.copy(cam_sensor.raw_data), (cam_sensor.height, cam_sensor.width, 4))
                cv2.imshow("CAM_FRONT", cam_sensor_image)
                location = validation_vehicle.get_location()
                actual_path.append((location.x, location.y, location.z))
                if validation_vehicle_control_agent.done():
                    print(f"{datetime.now()} Checkpoint reached. Validation vehicle has reached {len(episode_ego_vehicle_path) - len(agent_path)}/{len(episode_ego_vehicle_path)} planned path points.")
                    if (agent_path == []):
                        agent_path_completed = True
                        break
                    next_wp = agent_path.pop(0)
                    validation_vehicle_control_agent.set_destination(next_wp)
                elif next_wp is not None and next_wp.distance(validation_vehicle.get_location()) < 2.0:
                    print(f"{datetime.now()} Checkpoint reached. Validation vehicle has reached {len(episode_ego_vehicle_path) - len(agent_path)}/{len(episode_ego_vehicle_path)} planned path points.")
                    if (agent_path == []):
                        agent_path_completed = True
                        break
                    next_wp = agent_path.pop(0)
                    validation_vehicle_control_agent.set_destination(next_wp)
                validation_vehicle.apply_control(validation_vehicle_control_agent.run_step())
            print("Drive finished!")
    except RuntimeError as error:
        print(error)
    finally:
        cv2.destroyAllWindows()
        validation_vehicle.destroy()

    assert agent_path_completed, "Validation failed. The vehicle could not complete the planned path."
    return episode_ego_vehicle_path, actual_path

validation_results = dict()
if not SKIP_VALIDATION:
    print("Validating the episode path...")
    episode_ego_vehicle_path, actual_path = validate_path()
    validation_results["planned_path"] = episode_ego_vehicle_path
    validation_results["actual_path"] = actual_path
    print("Validation path successfully completed.")

def filter_path(coords, min_distance):
    """Filter path points to ensure minimum distance between them."""
    def distance(point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    filtered_coords = []
    for i in range(len(coords)):
        if (i == 0) or (i == (len(coords)-1)) or distance(filtered_coords[-1], coords[i]) >= min_distance:
            filtered_coords.append(coords[i])
    return filtered_coords

def visualize_paths(planned_path, actual_path):
    """Visualize planned and actual paths using Pygame with zoom/pan and filtering."""
    pygame.init()
    WIDTH, HEIGHT = 800, 600
    BACKGROUND_COLOR = (255, 255, 255)
    NODE_COLOR = (0, 0, 255)
    EDGE_COLOR = (0, 0, 0)
    PLANNED_NODE_COLOR = (0, 255, 0)
    PLANNED_EDGE_COLOR = (0, 200, 0)
    NODE_RADIUS = 5
    ARROW_SIZE = 10
    FONT_COLOR = (0, 0, 0)
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Validation path summary")

    scale = 1.0
    offset_x, offset_y = 0, 0
    zoom_factor = 1.1
    dragging = False
    last_mouse_pos = (0, 0)
    show_actual_path = True
    show_planned_path = True
    actual_path_distance_filter = 10.0
    planned_path_distance_filter = 10.0

    def draw_arrow(surface, start, end, color):
        pygame.draw.line(surface, color, start, end, 2)
        angle = math.atan2(end[1] - start[1], end[0] - start[0])
        left_angle = angle + math.pi / 6
        right_angle = angle - math.pi / 6
        left_point = (end[0] - ARROW_SIZE * math.cos(left_angle),
                      end[1] - ARROW_SIZE * math.sin(left_angle))
        right_point = (end[0] - ARROW_SIZE * math.cos(right_angle),
                       end[1] - ARROW_SIZE * math.sin(right_angle))
        pygame.draw.polygon(surface, color, [end, left_point, right_point])

    def draw_nodes_and_edges(coords, node_color, edge_color):
        for i, (x, y, z) in enumerate(coords):
            scaled_x = (x + offset_x) * scale
            scaled_y = (y + offset_y) * scale
            pygame.draw.circle(screen, node_color, (int(scaled_x), int(scaled_y)), NODE_RADIUS)
            font = pygame.font.Font(None, 24)
            text_surface = font.render(str(i), True, FONT_COLOR)
            screen.blit(text_surface, (scaled_x + 10, scaled_y - 10))
            if i < len(coords) - 1:
                next_x, next_y, _ = coords[i + 1]
                next_scaled_x = (next_x + offset_x) * scale
                next_scaled_y = (next_y + offset_y) * scale
                draw_arrow(screen, (int(scaled_x), int(scaled_y)), (int(next_scaled_x), int(next_scaled_y)), edge_color)

    def draw_checkbox(x, y, label, checked):
        pygame.draw.rect(screen, (0, 0, 0), (x, y, 20, 20), 2)
        if checked:
            pygame.draw.rect(screen, (0, 255, 0), (x + 2, y + 2, 16, 16))
        font = pygame.font.Font(None, 24)
        text_surface = font.render(label, True, FONT_COLOR)
        screen.blit(text_surface, (x + 30, y))

    def draw_slider(x, y, value):
        pygame.draw.rect(screen, (200, 200, 200), (x, y, 200, 20))
        pygame.draw.rect(screen, (100, 100, 100), (x + int(value), y, 10, 20))

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    scale *= zoom_factor
                elif event.button == 5:
                    scale /= zoom_factor
                elif event.button == 1:
                    dragging = True
                    last_mouse_pos = pygame.mouse.get_pos()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    dragging = False
                mouse_pos = pygame.mouse.get_pos()
                if 10 <= mouse_pos[0] <= 30 and 10 <= mouse_pos[1] <= 30:
                    show_actual_path = not show_actual_path
                if 10 <= mouse_pos[0] <= 30 and 90 <= mouse_pos[1] <= 110:
                    show_planned_path = not show_planned_path
            elif event.type == pygame.MOUSEMOTION:
                mouse_on_slider = False
                if dragging:
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    if (10 <= mouse_x <= 210) and (40 <= mouse_y <= 80):
                        actual_path_distance_filter = (mouse_x - 10) 
                        mouse_on_slider = True
                    if (10 <= mouse_x <= 210) and (120 <= mouse_y <= 160):
                        planned_path_distance_filter = (mouse_x - 10)
                        mouse_on_slider = True
        if dragging:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            if not mouse_on_slider:
                offset_x -= (last_mouse_pos[0] - mouse_x) / scale
                offset_y -= (last_mouse_pos[1] - mouse_y) / scale
            last_mouse_pos = (mouse_x, mouse_y)

        screen.fill(BACKGROUND_COLOR)
        draw_checkbox(10, 10, "Show Actual Path", show_actual_path)
        draw_slider(10, 50, actual_path_distance_filter)
        draw_checkbox(10, 90, "Show Planned Path", show_planned_path)
        draw_slider(10, 130, planned_path_distance_filter)
        if show_actual_path:
            filtered_actual_path = filter_path(actual_path, actual_path_distance_filter)
            draw_nodes_and_edges(filtered_actual_path, NODE_COLOR, EDGE_COLOR)
        if show_planned_path:
            filtered_planned_path = filter_path(planned_path, planned_path_distance_filter)
            draw_nodes_and_edges(filtered_planned_path, PLANNED_NODE_COLOR, PLANNED_EDGE_COLOR)
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()

if not SKIP_VALIDATION:
    print("Displaying the validation path summary...")
    visualize_paths(validation_results["planned_path"], validation_results["actual_path"])
    print("Validation path summary closed!")
    print("Continuing with full simulation.")

# --- Prepare CARLA world for full simulation ---

print("Setting up the full simulation environment...")
# reload_world()

# Spawn ego vehicle
agent_path = get_agent_path(episode_ego_vehicle_path)

blueprint_name = "vehicle.dodge.charger_2020"
blueprint = blueprint_library.find(blueprint_name)
blueprint.set_attribute('role_name','ego')
transform = find_nearest_spawn_point(agent_path[0])
vehicle = world.spawn_actor(blueprint, transform)

sensor_names = []
sensor_types = []
sensors = []

# Spawn sensors based on URDF and intrinsics
for sensor_configuration in ego_vehicle_extrinsics.robot.links:
    sensor_name = sensor_configuration.name
    if "CAM_" in sensor_name:
        blueprint_name = "sensor.camera.rgb"
        sensor_type = blueprint_name
        blueprint = blueprint_library.find(blueprint_name)
        sensor_intrinsics = ego_vehicle_intrinsics.get(sensor_name, dict())
        def calculate_fov(focal_length, image_width):
            fov_radians = 2 * np.arctan(image_width / (2 * focal_length))
            fov_degrees = np.degrees(fov_radians)
            return fov_degrees
        image_width = str(sensor_intrinsics.get("w", 1600))
        blueprint.set_attribute('image_size_x', image_width)
        image_height = str(sensor_intrinsics.get("h", 900))
        blueprint.set_attribute('image_size_y', image_height)
        focal_distance = float(sensor_intrinsics.get("fl"))
        field_of_view = str(calculate_fov(focal_distance, float(image_width)))
        blueprint.set_attribute('fov', field_of_view)
        transform_matrix = ego_vehicle_extrinsics.get_T_from_to("base_link", sensor_name)
        transform_matrix = transform_to_carla(sensor_type, transform_matrix)
        transform = matrix_to_transform(transform_matrix)
        attached_to = vehicle
        sensor = world.spawn_actor(blueprint, transform, attach_to=attached_to)
        sensors.append(sensor)
        sensor_types.append(sensor_type)
        sensor_names.append(sensor_name)
        # Spawn additional DEPTH and SEMANTIC cameras
        blueprint_name = "sensor.camera.depth"
        sensor_type = blueprint_name
        blueprint = blueprint_library.find(blueprint_name)
        blueprint.set_attribute('image_size_x', image_width)
        blueprint.set_attribute('image_size_y', image_height)
        blueprint.set_attribute('fov', field_of_view)
        attached_to = vehicle
        sensor = world.spawn_actor(blueprint, transform, attach_to=attached_to)
        sensors.append(sensor)
        sensor_types.append(sensor_type)
        sensor_names.append(f"DEPTH_{sensor_name}")
        blueprint_name = "sensor.camera.instance_segmentation"
        sensor_type = blueprint_name
        blueprint = blueprint_library.find(blueprint_name)
        blueprint.set_attribute('image_size_x', image_width)
        blueprint.set_attribute('image_size_y', image_height)
        blueprint.set_attribute('fov', field_of_view)
        attached_to = vehicle
        sensor = world.spawn_actor(blueprint, transform, attach_to=attached_to)
        sensors.append(sensor)
        sensor_types.append(sensor_type)
        sensor_names.append(f"SEMANTIC_{sensor_name}")
    elif "RADAR_" in sensor_name:
        blueprint_name = "sensor.other.radar"
        sensor_type = blueprint_name
        blueprint = blueprint_library.find(blueprint_name)     
        blueprint.set_attribute('horizontal_fov', str(30.0)) 
        blueprint.set_attribute('vertical_fov', str(30.0)) 
        blueprint.set_attribute('points_per_second', str(1e5))
        transform_matrix = ego_vehicle_extrinsics.get_T_from_to("base_link", sensor_name)
        transform_matrix = transform_to_carla(sensor_type, transform_matrix)
        transform = matrix_to_transform(transform_matrix)
        attached_to = vehicle
        sensor = world.spawn_actor(blueprint, transform, attach_to=attached_to)
        sensors.append(sensor)
        sensor_types.append(sensor_type)
        sensor_names.append(sensor_name)
    elif "LIDAR_" in sensor_name:
        blueprint_name = "sensor.lidar.ray_cast_semantic"
        sensor_type = blueprint_name
        blueprint = blueprint_library.find(blueprint_name)
        blueprint.set_attribute("channels", str(128))
        blueprint.set_attribute("points_per_second", str(2000000))
        blueprint.set_attribute("range", str(100))
        blueprint.set_attribute("rotation_frequency", str(20))
        blueprint.set_attribute("upper_fov", str(30))
        blueprint.set_attribute("lower_fov", str(-30))
        transform_matrix = ego_vehicle_extrinsics.get_T_from_to("base_link", sensor_name)
        transform_matrix = transform_to_carla(sensor_type, transform_matrix)
        transform = matrix_to_transform(transform_matrix)
        attached_to = vehicle
        sensor = world.spawn_actor(blueprint, transform, attach_to=attached_to)
        sensors.append(sensor)
        sensor_types.append(sensor_type)
        sensor_names.append(sensor_name)

print(f"Number of sensors spawned: {len(sensors)}")
print(sensor_names)
print(sensor_types)

# --- Add traffic (vehicles and pedestrians) ---

def add_vehicles_to_simulation(n_vehicles: int=0):
    """Spawn non-ego vehicles and set them to autopilot."""
    print(f"Spawning {n_vehicles} vehicles...")
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    all_actors = []
    spawn_points = world.get_map().get_spawn_points()
    while (len(all_actors) < n_vehicles):
        vehicle_bp = random.choice(vehicle_blueprints)
        vehicle = None
        while vehicle is None:
            spawn_point = random.choice(spawn_points)
            vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
            time.sleep(0.1)
        all_actors.append(vehicle)
    port = traffic_manager.get_port()
    for actor in all_actors:
        actor.set_autopilot(True, port)
    print(f"Spawned {len(all_actors)} vehicles!")
    return all_actors

def add_pedestrians_to_simulation(n_pedestrians: int=0):
    """Spawn pedestrians and assign AI controllers."""
    print(f"Spawning {n_pedestrians} pedestrians...")
    pedestrian_blueprints = blueprint_library.filter('walker.pedestrian.*')
    walker_controller_bp = blueprint_library.find('controller.ai.walker')
    all_pedestrians = []
    all_ai_controllers = []
    spawn_points = world.get_map().get_spawn_points()
    while (len(all_pedestrians) < n_pedestrians):
        pedestrian_bp = random.choice(pedestrian_blueprints)        
        pedestrian = None
        while pedestrian is None:
            spawn_point = random.choice(spawn_points)
            pedestrian = world.try_spawn_actor(pedestrian_bp, spawn_point)
            time.sleep(0.1)
        all_pedestrians.append(pedestrian)
        walker_controller = None
        while walker_controller is None:
            walker_controller = world.try_spawn_actor(walker_controller_bp, carla.Transform(), pedestrian)
            time.sleep(0.1)
        all_ai_controllers.append(walker_controller)
        walker_controller.start()
        walker_controller.go_to_location(world.get_random_location_from_navigation())
        walker_controller.set_max_speed(1 + random.random())
    print(f"Spawned {len(all_pedestrians)} pedestrians!")
    return all_pedestrians, all_ai_controllers

if N_VEHICLES > 0:
    add_vehicles_to_simulation(n_vehicles=N_VEHICLES)
if N_PEDESTRIANS > 0:
    add_pedestrians_to_simulation(n_pedestrians=N_PEDESTRIANS)
print("Simulation environment setup completed!")

# --- Optionally toggle off all buildings ---

if TOGGLE_OFF_BUILDINGS:
    print("Turing off buildings...")
    objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)
    building_ids = [obj.id for obj in objs]
    world.enable_environment_objects(building_ids, False)
    print("Buildings are turned off.")

# --- Filesystem and sensor data utilities ---

def delete_all_in_directory(target_directory):
    """Delete all files and folders in the target directory."""
    if os.path.exists(target_directory):
        for filename in os.listdir(target_directory):
            file_path = os.path.join(target_directory, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.remove(file_path) 
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                print(f'Failed to delete {file_path}. Reason: {e}')

def create_filename_from_timestamp(timestamp):
    """Create a filename based on a timestamp (in nanoseconds)."""
    SECONDS_TO_NANOSECONDS = 1000000000
    filename = str(math.trunc(timestamp * SECONDS_TO_NANOSECONDS))
    return filename

# --- Sensor data processing and saving ---

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
    filename = create_filename_from_timestamp(raw_data.timestamp) + ".npy"
    filepath = os.path.join(target_directory, filename)
    np.save(filepath, transform_matrix)

def save_camera_image(raw_data, target_directory):    
    """Save a camera image (RGB, depth, or semantic) as PNG."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    rgb_image = np.reshape(raw_data.raw_data, (raw_data.height, raw_data.width, 4))[:, :, :3]
    filename = create_filename_from_timestamp(raw_data.timestamp) + ".png"
    filepath = os.path.join(target_directory, filename)
    cv2.imwrite(filepath, rgb_image)

def save_radar_readings(raw_data, target_directory):
    """Save radar point cloud as .ply file."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    radar_points_list = []
    for measurement in raw_data:
        azi = math.degrees(measurement.azimuth)
        alt = math.degrees(measurement.altitude)
        fw_vec = carla.Vector3D(x=measurement.depth)
        carla.Transform(
            carla.Location(),
            carla.Rotation(pitch=alt,yaw=azi,roll=0)
        ).transform(fw_vec)
        radar_points_list.append([fw_vec.x, fw_vec.y, fw_vec.z])
    points = np.array(radar_points_list)
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud = convert_point_cloud_from_left_to_right_hand_system(point_cloud)
    filename = create_filename_from_timestamp(raw_data.timestamp) + ".ply"
    filepath = os.path.join(target_directory, filename)
    o3d.io.write_point_cloud(filepath, point_cloud)

def save_lidar_readings(raw_data, target_directory):
    """Save lidar point cloud as .ply file."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    points = np.array([(detection.point.x, detection.point.y, detection.point.z) for detection in raw_data])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud = convert_point_cloud_from_left_to_right_hand_system(point_cloud)
    filename = create_filename_from_timestamp(raw_data.timestamp) + ".ply"
    filepath = os.path.join(target_directory, filename)
    o3d.io.write_point_cloud(filepath, point_cloud)

def save_semantic_lidar_readings(raw_data, target_directory):
    """Save semantic lidar point cloud with object tags as .ply file."""
    if not os.path.exists(target_directory):
        os.makedirs(target_directory, exist_ok=True)
    lidar_data = np.array([(detection.point.x, detection.point.y, detection.point.z, detection.object_tag) for detection in raw_data])
    points = lidar_data[:, :3]
    semantic_tags = lidar_data[:, 3] 
    semantic_colors_rgb = np.zeros((len(semantic_tags), 3))
    semantic_colors_rgb[:, 0] = semantic_tags
    semantic_colors_normalized = semantic_colors_rgb / 255.0
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(semantic_colors_normalized)
    point_cloud = convert_point_cloud_from_left_to_right_hand_system(point_cloud)
    filename = create_filename_from_timestamp(raw_data.timestamp) + ".ply"
    filepath = os.path.join(target_directory, filename)
    o3d.io.write_point_cloud(filepath, point_cloud)

# --- Main simulation loop ---

delete_all_in_directory(SIMULATION_DATA_OUTPUT_PATH)

print("Starting the simulation...")
cv2.namedWindow("Press Q to stop simulation")
cv2.imshow("Press Q to stop simulation", np.zeros((1,1)))

vehicle_control_agent = BasicAgent(vehicle, target_speed=15)
agent_path = get_agent_path(episode_ego_vehicle_path)
vehicle_control_agent.set_destination(agent_path.pop(0))

try:
    with CarlaSyncMode(world, sensors, fps=10) as sync_mode:
        print("Driving to the start of the path...")
        while vehicle_control_agent.done() is False:
            simulated_results = sync_mode.tick()[1:]
            if "CAM_FRONT" in sensor_names:
                front_cam_sensor = simulated_results[sensor_names.index("CAM_FRONT")]
                front_cam_sensor_image =  np.reshape(np.copy(front_cam_sensor.raw_data), (front_cam_sensor.height, front_cam_sensor.width, 4))
                cv2.imshow("Driving to start of the path...", front_cam_sensor_image)
            vehicle.apply_control(vehicle_control_agent.run_step())
            if (cv2.waitKey(1) == ord('q')):
                break
        cv2.destroyWindow("Driving to start of the path...")
        print("Arrived at the start of the path!")

        print("Driving on path...")
        while True:
            simulation_results = sync_mode.tick(timeout=100.0)[1:]
            for i in range(len(simulation_results)):
                sensor = sensors[i]
                sensor_data = simulation_results[i]
                sensor_name = sensor_names[i].replace("base_link_to_", "")
                sensor_type = sensor_types[i]
                sensor_data_path = os.path.join(SIMULATION_DATA_OUTPUT_PATH, sensor_name)
                if ("camera.rgb" in sensor_type) or ("camera.instance_segmentation" in sensor_type) or ("camera.depth" in sensor_type):
                    save_camera_image(sensor_data, sensor_data_path)
                elif ("sensor.other.radar" in sensor_type):
                    save_radar_readings(sensor_data, sensor_data_path)
                elif ("sensor.lidar.ray_cast_semantic" in sensor_type):
                    save_semantic_lidar_readings(sensor_data, sensor_data_path)
                elif ("sensor.lidar" in sensor_type):
                    save_lidar_readings(sensor_data, sensor_data_path)
                save_sensor_position(sensor_data, sensor_data_path, sensor_type=sensor_type)
            if vehicle_control_agent.done():
                print(f"{datetime.now()} Checkpoint reached. Ego vehicle has reached {len(episode_ego_vehicle_path) - len(agent_path)}/{len(episode_ego_vehicle_path)} planned path points.")
                if len(agent_path) == 0:
                    break
                vehicle_control_agent.set_destination(agent_path.pop(0))
            vehicle.apply_control(vehicle_control_agent.run_step())
            if cv2.waitKey(1) == ord('q'):
                break
        print("Driving on path finished!")
except RuntimeError as error:
    print("An error occurred during the simulation.")
    print(error)
finally:
    cv2.destroyAllWindows()
    for sensor in sensors:
        sensor.stop()
        sensor.destroy()
    vehicle.destroy()

print("Simulation finished!")
