import open3d as o3d
import numpy as np

# Load the point cloud from a PLY file
import os 


current_dir = os.getcwd()

data_dir = os.path.join(current_dir, 'src', 'generated_data', 'RADAR_FRONT')

stuff = []

for file in os.listdir(data_dir):
    file_path = os.path.join(data_dir, file)
    
    # Check if the file is a .npy file
    
    print(file_path)
    if file.endswith('.npy'):
        matrix = np.load(file_path)
        ply_file = file_path.replace('.npy', '.ply')
        point_cloud = o3d.io.read_point_cloud(ply_file)
        point_cloud.transform(matrix)
        
        # create axis frame
        axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.9, origin=[0, 0, 0])
        axis_frame.transform(matrix)
        
        stuff += [point_cloud, axis_frame]
        
                
o3d.visualization.draw_geometries(stuff)