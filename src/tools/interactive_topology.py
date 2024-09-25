import pygame
import math
import carla

# Import map from Carla
client = carla.Client('localhost', 2000)
client.set_timeout(10)
world = client.get_world()
map = world.get_map()

# Initialize pygame
pygame.init()

# Set up display
width, height = 1200, 900
window = pygame.display.set_mode((width, height))
node_pos_scaler = 3
node_pos_shifter = 0
screen_offset_x = 0
screen_offset_y = 0
screen_offset_shift = 5
pygame.display.set_caption("Directed Graph with Hover Labels")

# Define colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GRAY = (100, 100, 100)
BLUE = (0, 0, 255)

# Choose waypoints
selected_edges = set()
path = []

# Define graph structure
topology = map.get_topology()
nodes = {}
edges = []
street_waypoints = {}
x_min = float("inf")
y_min = float("inf")

for wp1, wp2 in topology:
    loc1, loc2 = wp1.transform.location, wp2.transform.location
    wp1_x, wp1_y, wp1_z = loc1.x, loc1.y, loc1.z
    wp2_x, wp2_y, wp2_z = loc2.x, loc2.y, loc2.z

    street_x = (wp1_x + wp2_x) / 2
    street_y = (wp1_y + wp2_y) / 2
    street_waypoints[(wp1.id, wp2.id)] = (street_x, street_y)

    x_min = min(x_min, wp2_x, wp2_x)
    y_min = min(y_min, wp1_y, wp2_y)

    nodes[wp1.id] = (wp1_x, wp1_y, wp1_z)
    nodes[wp2.id] = (wp2_x, wp2_y, wp2_z)

    edges.append((wp1.id, wp2.id))

# nodes = {
#     'A': (100, 100),
#     'B': (300, 100),
#     'C': (500, 100),
#     'D': (200, 300),
#     'E': (400, 300)
# }

# edges = [
#     ('A', 'B'),
#     ('B', 'C'),
#     ('A', 'D'),
#     ('D', 'E'),
#     ('E', 'C')
# ]

# Function to draw directed edges
def draw_edges():
    for start, end in edges:
        color = RED if (start, end) in selected_edges else GRAY
        
        start_pos = nodes[start]
        start_x = start_pos[0] * node_pos_scaler + node_pos_shifter + screen_offset_x
        start_y = start_pos[1] * node_pos_scaler + node_pos_shifter + screen_offset_y
        
        end_pos = nodes[end]
        end_x = end_pos[0] * node_pos_scaler + node_pos_shifter + screen_offset_x
        end_y = end_pos[1] * node_pos_scaler + node_pos_shifter + screen_offset_y
        pygame.draw.line(window, color, (start_x, start_y), (end_x, end_y), 2)
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


def get_closest_street(x, y):
    closest = None
    closest_distance = None
    for node, (street_x, street_y) in street_waypoints.items():
        distance = get_distance(x, y, street_x, street_y)
        if closest is None or distance < closest_distance:
            closest = node
            closest_distance = distance
    return closest

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
                street = get_closest_street(x, y)
                selected_edges.add(street)
                start, end = street
                if not path:
                    path.append(nodes[start])
                path.append(nodes[end])
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
print(path)
