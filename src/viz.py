import os
import numpy as np
import open3d as o3d

# Get the current directory
current_dir = os.getcwd()
data_dir = os.path.join(current_dir, 'src', 'generated_data')

# Create an Open3D visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()


filename = None
# Iterate over all subdirectories in the current directory
for folder in os.listdir(data_dir):
    folder_path = os.path.join(data_dir, folder)
    
    # Check if the item is a directory
    if os.path.isdir(folder_path):
        # Look for .npy files in the directory
        for file in os.listdir(folder_path):
            file_path = os.path.join(folder_path, file)
            
            # Check if the file is a .npy file
            if file.endswith('.npy'):
                if filename is None:
                    filename = os.path.basename(file_path)
  
                
                # Load the .npy file as a 4x4 transformation matrix
                # print(file_path)
                file_path = os.path.join(folder_path, filename)
                matrix = np.load(file_path)
                
                lh_matrix = np.array([
                    [1, 0, 0, 0],
                    [0, -1, 0, 0],  # Flip y-axis
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ])
                matrix = lh_matrix @ matrix
                
                # Create an Open3D geometry from the transformation matrix
                frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
                frame.transform(matrix)
                
                # Add the frame to the visualization
                vis.add_geometry(frame)
                
                # Create a text label for the folder name
                text_mesh = o3d.t.geometry.TriangleMesh.create_text(folder, depth=0.1).to_legacy()
                text_mesh.paint_uniform_color((1, 0, 0))
                
                # Scale down and position the text
                scale = 0.005
                text_mesh.transform([[scale, 0, 0, matrix[0, 3]], 
                                     [0, scale, 0, matrix[1, 3]], 
                                     [0, 0, scale, matrix[2, 3]], 
                                     [0, 0, 0, 1]])
                
                # Enable back-face rendering
                text_mesh.compute_vertex_normals()
                vis.get_render_option().mesh_show_back_face = True
                
                # Add the text label to the visualization
                vis.add_geometry(text_mesh)

# Run the visualization
vis.run()

# Close the visualization window
vis.destroy_window()