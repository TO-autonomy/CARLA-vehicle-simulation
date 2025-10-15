import json
import toml
import xml.etree.ElementTree as ET

# File paths
intrinsics_file = "/home/leppsalu/Desktop/rework/CARLA-vehicle-simulation/src/config/carla_intrinsics.json"
extrinsics_file = "/home/leppsalu/Desktop/rework/CARLA-vehicle-simulation/src/config/carla_extrinsics.urdf"
output_toml_file = "/home/leppsalu/Desktop/rework/CARLA-vehicle-simulation/src/config/sensors.toml"

# Load intrinsics JSON
with open(intrinsics_file, "r") as f:
    intrinsics_data = json.load(f)

# Parse extrinsics URDF
tree = ET.parse(extrinsics_file)
root = tree.getroot()

# Extract extrinsics data
extrinsics_data = {}
for joint in root.findall("joint"):
    child = joint.find("child").attrib["link"]
    origin = joint.find("origin")
    xyz = [float(x) for x in origin.attrib["xyz"].split()]
    rpy = [float(r) for r in origin.attrib["rpy"].split()]
    extrinsics_data[child] = {"position": xyz, "rotation": rpy}

# Combine intrinsics and extrinsics into TOML format
sensors = {}
for sensor_name, intrinsic in intrinsics_data.items():
    sensor_data = {
        "focal_length": intrinsic["fl"],
        "image_size_x": intrinsic["w"],
        "image_size_y": intrinsic["h"],
        "principal_point_x": intrinsic["ppx"],
        "principal_point_y": intrinsic["ppy"],
        "distortion_coefficients": intrinsic["disto"],
        "distortion_type": intrinsic["disto_type"],
    }
    if sensor_name in extrinsics_data:
        sensor_data["position"] = extrinsics_data[sensor_name]["position"]
        sensor_data["rotation"] = extrinsics_data[sensor_name]["rotation"]
    sensors[sensor_name] = sensor_data

# Write to TOML file
with open(output_toml_file, "w") as f:
    toml.dump({"sensors": sensors}, f)

print(f"Converted data has been saved to {output_toml_file}")