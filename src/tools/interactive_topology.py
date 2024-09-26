import pygame
import math
import carla
import json
import time
import re
from pathlib import Path

# Import map from Carla
client = carla.Client('localhost', 2000)
client.set_timeout(10)
# world = client.load_world("Town03")
world = client.get_world()
map = world.get_map()

# Save to file
dir = "src/routes"
map_name = re.findall(r"Town\d+", map.name)[0]
filename = f"{map_name}_{int(time.time())}.json"

# Initialize pygame
pygame.init()

# Set up display
width, height = 1200, 900
window = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
clock.tick(30)
node_pos_scaler = 3
node_pos_shifter = 0
screen_offset_x = 0
screen_offset_y = 0
screen_offset_shift = 5
pygame.display.set_caption("Create route")

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY = (100, 100, 100)
BLUE = (0, 0, 255)

# Choose waypoints
selected_edges = set()
path = []
waypoint_indices = []
distance_threshold = 0.8 * node_pos_scaler

# Define graph structure
topology = map.get_topology()
nodes = {}
edges = []
street_waypoints = {}

for wp1, wp2 in topology:
    loc1, loc2 = wp1.transform.location, wp2.transform.location
    wp1_x, wp1_y, wp1_z = loc1.x, loc1.y, loc1.z
    wp2_x, wp2_y, wp2_z = loc2.x, loc2.y, loc2.z

    street_x = (wp1_x + wp2_x) / 2
    street_y = (wp1_y + wp2_y) / 2
    street_waypoints[(wp1.id, wp2.id)] = (street_x, street_y)

    nodes[wp1.id] = (wp1_x, wp1_y, wp1_z)
    nodes[wp2.id] = (wp2_x, wp2_y, wp2_z)

    edges.append((wp1.id, wp2.id))

# Function to draw directed edges
def draw_edges():
    for start, end in edges:
        color = GREEN if (start, end) in selected_edges else GRAY
        
        start_pos = nodes[start]
        start_x = start_pos[0] * node_pos_scaler + node_pos_shifter + screen_offset_x
        start_y = start_pos[1] * node_pos_scaler + node_pos_shifter + screen_offset_y
        
        end_pos = nodes[end]
        end_x = end_pos[0] * node_pos_scaler + node_pos_shifter + screen_offset_x
        end_y = end_pos[1] * node_pos_scaler + node_pos_shifter + screen_offset_y
        pygame.draw.line(window, color, (start_x, start_y), (end_x, end_y), 4)
        draw_arrowhead((start_x, start_y), (end_x, end_y))

def draw_street_nodes():
    for node, (x, y) in street_waypoints.items():
        x = x * node_pos_scaler + node_pos_shifter + screen_offset_x
        y = y * node_pos_scaler + node_pos_shifter + screen_offset_y
        pygame.draw.circle(window, RED, (x, y), 5)

# Function to draw arrowheads
def draw_arrowhead(start, end):
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    arrow_length = 10
    arrow_angle = math.pi / 6
    x1 = end[0] - arrow_length * math.cos(angle - arrow_angle)
    y1 = end[1] - arrow_length * math.sin(angle - arrow_angle)
    x2 = end[0] - arrow_length * math.cos(angle + arrow_angle)
    y2 = end[1] - arrow_length * math.sin(angle + arrow_angle)
    pygame.draw.polygon(window, BLUE, [(end[0], end[1]), (x1, y1), (x2, y2)])

# Function to check if mouse is over a node
def is_mouse_over_node(mouse_pos, node_pos):
    return math.hypot(mouse_pos[0] - node_pos[0], mouse_pos[1] - node_pos[1]) < 20


def get_distance(x, y, street_x, street_y):
    street_x = street_x * node_pos_scaler + node_pos_shifter + screen_offset_x
    street_y = street_y * node_pos_scaler + node_pos_shifter + screen_offset_y
    return ((street_x - x) ** 2 + (street_y - y) ** 2) ** 0.5


def get_street(x, y):
    for node, (street_x, street_y) in street_waypoints.items():
        distance = get_distance(x, y, street_x, street_y)
        if distance < distance_threshold:
            return node

# Main loop
running = True
while running:
    mouse_pos = pygame.mouse.get_pos()
    window.fill(WHITE)
    draw_edges()
    draw_street_nodes()

    for event in pygame.event.get():
        pressed = pygame.key.get_pressed()
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                x, y = pygame.mouse.get_pos()
                street = get_street(x, y)
                if street:
                    selected_edges.add(street)
                    start, end = street
                    if not waypoint_indices or waypoint_indices[-1] != end:
                        if not path:
                            path.append(nodes[start])
                            waypoint_indices.append(start)
                        path.append(nodes[end])
                        waypoint_indices.append(end)
    if pressed[pygame.K_LEFT]:
        screen_offset_x += screen_offset_shift
    elif pressed[pygame.K_RIGHT]:
        screen_offset_x -= screen_offset_shift
    elif pressed[pygame.K_UP]:
        screen_offset_y += screen_offset_shift
    elif pressed[pygame.K_DOWN]:
        screen_offset_y -= screen_offset_shift

    pygame.display.flip()

pygame.quit()
if path:
    Path(dir).mkdir(parents=True, exist_ok=True)
    route_dict = {
        "map": map_name,
        "waypoint_indices": waypoint_indices,
        "route": path
    }
    json_obj = json.dumps(route_dict, indent=4)
    with open(f"{dir}/{filename}", "w") as f:
        f.write(json_obj)