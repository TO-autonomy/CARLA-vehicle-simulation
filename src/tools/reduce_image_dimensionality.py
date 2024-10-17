import os
import argparse
from PIL import Image

def convert_rgba_to_rgb(target_directory):
    """
    Convert all RGBA images in the target directory and its subdirectories to RGB.

    Parameters:
    - target_directory: The path to the directory containing image files.
    """
    # Traverse the directory tree
    for dirpath, _, filenames in os.walk(target_directory):
        for filename in filenames:
            # Construct full file path
            file_path = os.path.join(dirpath, filename)

            # Check if the file is an image based on the extension
            if filename.lower().endswith('.png'):
                with Image.open(file_path) as img:
                    try:
                        # Check if the image has an alpha channel (4 channels)
                        if img.mode == 'RGBA':
                            # Convert to RGB
                            rgb_img = img.convert('RGB')
                            # Save the converted image (you can choose to overwrite or save with a new name)
                            rgb_img.save(file_path)
                            print(f"Converted {filename} to RGB in {dirpath}.")
                    except Exception as e:
                        print(f"Error converting {filename}: {e}")
                        img.save(file_path)
                        return

def main():
    # Create an argument parser
    parser = argparse.ArgumentParser(description="Convert RGBA images to RGB in the specified directory.")
    
    # Add an argument for the target directory
    parser.add_argument('target_directory', type=str, help='Path to the directory containing image files')
    
    # Parse the arguments
    args = parser.parse_args()
    
    # Call the function with the provided target directory
    convert_rgba_to_rgb(args.target_directory)

if __name__ == "__main__":
    main()