import pygame
import math
import carla
import json
import time
import re
from pathlib import Path

class CARLAWaypoint:
    def __init__(self, waypoint):
        self._waypoint = waypoint
        self._id = waypoint.id
        self._location = waypoint.transform.location

    def get_id(self):
        return self._id
    
    def get_location(self):
        return (self._location.x, self._location.y, self._location.z)
    
    def get_section_id(self):
        return self._waypoint.section_id
    
    def distance_from(self, wp2):
        x0, y0, z0 = self.get_location()
        x1, y1, z1 = wp2.get_location()
        return math.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
    
    def next(self, distance=1.0):
        return [CARLAWaypoint(next_waypoint) for next_waypoint in self._waypoint.next(distance)]
    
    def __str__(self) -> str:
        return f"Waypoint {self._id} at {self._location}"
    
    def __eq__(self, wp2: object) -> bool:
        if not isinstance(wp2, CARLAWaypoint):
            return False
        return self._id == wp2.get_id()
    
    def __hash__(self) -> int:
        return hash(self._id)

class CARLAMap:
    def __init__(self, carla_map):
        self._map = carla_map
        self._name = self._map.name
        self.nodes = set()
        self.edges = set()
        self._load_topology()

    def _load_topology(self, distance_between_waypoints=10.0):
        topology = self._map.get_topology()

        edges = set()
        for edge in topology:
            start, end = CARLAWaypoint(edge[0]), CARLAWaypoint(edge[1])
            current = start
            while current.distance_from(end) >= distance_between_waypoints:
                next = None
                for next_waypoint in current.next(distance_between_waypoints):
                    if next_waypoint.get_section_id() == end.get_section_id():
                        next = next_waypoint
                        break
                edges.add((current, next))
                current = next
            if current != end:
                edges.add((current, end))
        self.edges = edges
        
        nodes = set()
        for edge in self.edges:
            nodes.add(edge[0])
            nodes.add(edge[1])
        self.nodes = nodes
        

    def show_topology(self, window_size=(1200, 900), node_radius=5, arrow_size=10, highlighted_paths=dict()):        
        # Initialize Pygame
        pygame.init()

        # Constants
        WIDTH, HEIGHT = window_size[0], window_size[1]
        BACKGROUND_COLOR = (255, 255, 255)
        NODE_RADIUS = node_radius
        ARROW_SIZE = arrow_size
        FONT_COLOR = (0, 0, 0)

        # Create the screen
        screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(f"CARLA Topology Viewer ({self._name})")

        # Zoom and pan variables
        scale = 1.0
        offset_x, offset_y = 0, 0
        zoom_factor = 1.1
        dragging = False
        last_mouse_pos = (0, 0)

        # Town variables
        location_waypoint = dict()
        for i, waypoint in enumerate(self.nodes):
            location_waypoint[waypoint.get_location()] = waypoint 

        # Add town map to highlighted routes
        town_name = self._name
        town_map_edges = [(waypoint1, waypoint2) for waypoint1, waypoint2 in self.edges]
        highlighted_paths[town_name] = town_map_edges

        # Add new path to highlighted routes
        new_path_name = "ego vehicle route"

        # Checkbox and slider states
        path_visibility = {route_name: True for route_name in highlighted_paths}
        path_distance_filters = {route_name: 10.0 for route_name in highlighted_paths}

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

        def get_scaled_x(x):
            return (x + offset_x) * scale
        
        def get_descaled_x(scaled_x):
            return scaled_x / scale - offset_x
        
        def get_scaled_y(y):
            return (y + offset_y) * scale
        
        def get_descaled_y(scaled_y):
            return scaled_y / scale - offset_y
        
        def draw_nodes(nodes, node_color, show_ids=True):
            for i, (x, y, z) in enumerate(nodes):
                scaled_x = get_scaled_x(x)
                scaled_y = get_scaled_y(y)
                
                pygame.draw.circle(screen, node_color, (int(scaled_x), int(scaled_y)), NODE_RADIUS)
                
                if show_ids:
                    font = pygame.font.Font(None, 24)
                    text_surface = font.render(str(i), True, FONT_COLOR)
                    screen.blit(text_surface, (scaled_x + 10, scaled_y - 10))

        def draw_edges(edges, edge_color):
            for (x0, y0, z0), (x1, y1, z1) in edges:
                scaled_x0 = get_scaled_x(x0)
                scaled_y0 = get_scaled_y(y0)
                scaled_x1 = get_scaled_x(x1)
                scaled_y1 = get_scaled_y(y1)
                draw_arrow(screen, (int(scaled_x0), int(scaled_y0)), (int(scaled_x1), int(scaled_y1)), edge_color)

        def draw_checkbox(x, y, label, checked):
            pygame.draw.rect(screen, (0, 0, 0), (x, y, 20, 20), 2)
            if checked:
                pygame.draw.rect(screen, (0, 255, 0), (x + 2, y + 2, 16, 16))

            font = pygame.font.Font(None, 24)
            text_surface = font.render(label, True, FONT_COLOR)
            screen.blit(text_surface, (x + 30, y))

        def draw_slider(x, y, value):
            pygame.draw.rect(screen, (200, 200, 200), (x, y, 200, 20))  # Slider background
            pygame.draw.rect(screen, (100, 100, 100), (x + int(value), y, 10, 20))  # Slider handle

        # Main loop
        clock = pygame.time.Clock()
        running = True
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                
                # Zoom with mouse scroll wheel
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:  # Scroll up
                        scale *= zoom_factor
                    elif event.button == 5:  # Scroll down
                        scale /= zoom_factor
                    elif event.button == 1:  # Left mouse button
                        dragging = True
                        last_mouse_pos = pygame.mouse.get_pos()

                elif event.type == pygame.MOUSEBUTTONUP:
                    mouse_pos = pygame.mouse.get_pos()
                    if event.button == 1:  # Left mouse button
                        dragging = False
                        for i, route_name in enumerate(highlighted_paths):
                            if 10 <= mouse_pos[0] <= 30 and 10 + i * 80 <= mouse_pos[1] <= 30 + i * 80:
                                path_visibility[route_name] = not path_visibility[route_name]
                        
                    elif event.button == 3:  # Right mouse button
                        mouse_x, mouse_y = pygame.mouse.get_pos()
                        x, y = get_descaled_x(mouse_x), get_descaled_y(mouse_y)

                        shortest_distance = float('inf')
                        shortest_distance_node = None
                        for node in self.nodes:
                            x0, y0, z0 = node.get_location()
                            cursor_distance = ((x0 - x)**2 + (y0 - y)**2)**0.5
                            if cursor_distance < shortest_distance:
                                shortest_distance = cursor_distance
                                shortest_distance_node = node
                        if shortest_distance < NODE_RADIUS:
                            if highlighted_paths.get(new_path_name) is None:
                                highlighted_paths[new_path_name] = []
                                path_visibility[new_path_name] = True
                                path_distance_filters[new_path_name] = 10.0
                            highlighted_paths[new_path_name].append(shortest_distance_node.get_location())

                elif event.type == pygame.MOUSEMOTION:
                    mouse_on_slider = False
                    if dragging:
                        mouse_x, mouse_y = pygame.mouse.get_pos()
                        for i, path_name in enumerate(highlighted_paths):
                            if (10 <= mouse_x <= 210) and (40 + i * 80 <= mouse_y <= 80 + i * 80):  # Adjust value within slider range
                                path_distance_filters[path_name] = (mouse_x - 10) 
                                mouse_on_slider = True
                        
            # Drag to pan
            if dragging:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                if not mouse_on_slider:
                    offset_x -= (last_mouse_pos[0] - mouse_x) / scale
                    offset_y -= (last_mouse_pos[1] - mouse_y) / scale
                last_mouse_pos = (mouse_x, mouse_y)

            # Clear the screen
            screen.fill(BACKGROUND_COLOR)

            colors = [
                ((0, 0, 255), (173, 216, 230)),  # Blue nodes, Light Blue edges
                ((0, 255, 0), (144, 238, 144)),  # Green nodes, Light Green edges
                ((255, 0, 0), (255, 182, 193)),  # Red nodes, Light Red edges
                ((255, 255, 0), (255, 255, 224)),  # Yellow nodes, Light Yellow edges
                ((0, 255, 255), (224, 255, 255)),  # Cyan nodes, Light Cyan edges
            ]

            for i, (path_name, edges) in enumerate(highlighted_paths.items()):
                draw_checkbox(10, 10 + i * 80, f"Show {path_name}", path_visibility[path_name])
                draw_slider(10, 50 + i * 80, path_distance_filters[path_name])  # Scale for display

                if path_visibility[path_name]:
                    def filter_path(coords, min_distance):
                        def distance(point1, point2):
                            return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
                        filtered_coords = []
                        for i in range(len(coords)):
                            if (i == 0) or (i == (len(coords)-1)) or distance(filtered_coords[-1], coords[i]) >= min_distance:
                                filtered_coords.append(coords[i])
                        return filtered_coords
                    
                    
                    if path_name == self._name:
                        nodes = [waypoint.get_location() for waypoint in self.nodes]   
                        edges = [(waypoint1.get_location(), waypoint2.get_location()) for (waypoint1, waypoint2) in self.edges]
                        node_color = edge_color = (211, 211, 211) # Light Gray nodes and edges
                        show_ids = False
                    else:
                        path = highlighted_paths[path_name]
                        path = filter_path(path, path_distance_filters[path_name])
                        nodes = path
                        edges = [(path[i], path[i+1]) for i in range(len(path)-1)]
                        node_color, edge_color = colors[i % len(colors)]  # Cycle through colors
                        show_ids = True
                    draw_edges(edges, edge_color)
                    draw_nodes(nodes, node_color, show_ids=show_ids)

            # Update the display
            pygame.display.flip()
            clock.tick(60)

        output = dict()
        output["map"] = self._name.split("/")[-1]
        output["route"] = highlighted_paths[new_path_name]
        output_json = json.dumps(output, indent=4)
        print(output_json)

# Import map from Carla
client = carla.Client('localhost', 2000)
client.set_timeout(10)
town_name = "Town02"
world = client.load_world(town_name)
world = client.get_world()
world_map = world.get_map()

map = CARLAMap(world_map)
map.show_topology()


# Close Pygame
pygame.quit()