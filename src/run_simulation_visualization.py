import argparse
import os
import cv2
import numpy as np

def parse_arguments():
    parser = argparse.ArgumentParser(description="Generate a video from simulation data.")
    parser.add_argument("--layout", type=str, required=True, help="Grid layout for the video (e.g., 3-3-3).")
    parser.add_argument("--data_dir", type=str, required=True, help="Directory containing the simulation data.")
    parser.add_argument("--image_dirs", type=str, nargs='+', required=True, help="List of image directories to include in the video.")
    parser.add_argument("--output", type=str, default="output_video.mp4", help="Output video file name.")
    parser.add_argument("--fps", type=int, default=30, help="Frames per second for the video.")
    parser.add_argument("--resize", type=str, default=None, help="Resize the entire grid frame to a specific resolution (e.g., 1920x1080).")
    return parser.parse_args()

def load_images(data_dir, image_dirs):
    image_sequences = []
    for image_dir in image_dirs:
        dir_path = os.path.join(data_dir, image_dir)
        if not os.path.exists(dir_path):
            raise FileNotFoundError(f"Image directory not found: {dir_path}")
        images = sorted([os.path.join(dir_path, img) for img in os.listdir(dir_path) if img.endswith(('.png', '.jpg'))])
        image_sequences.append(images)
    return image_sequences

def create_video(layout, image_sequences, output_file, fps, resize=None):
    rows, cols = map(int, layout.split('-'))
    if len(image_sequences) != rows * cols:
        raise ValueError("Number of image directories does not match the layout.")

    # Determine frame size from the first image
    first_image = cv2.imread(image_sequences[0][0])
    if first_image is None:
        raise ValueError("Could not read the first image.")
    img_height, img_width, _ = first_image.shape

    # Calculate grid size
    grid_height = rows * img_height
    grid_width = cols * img_width

    # Initialize video writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (grid_width, grid_height))

    # Parse resize dimensions if provided
    if resize:
        try:
            resize_width, resize_height = map(int, resize.split('x'))
        except ValueError:
            raise ValueError("Invalid format for --resize. Use WIDTHxHEIGHT (e.g., 1920x1080).")

    # Iterate through frames
    for frame_idx in range(len(image_sequences[0])):
        grid_frame = np.zeros((grid_height, grid_width, 3), dtype=np.uint8)
        for i, images in enumerate(image_sequences):
            row, col = divmod(i, cols)
            img_path = images[frame_idx]
            img = cv2.imread(img_path)
            if img is None:
                raise ValueError(f"Could not read image: {img_path}")
            y_start, y_end = row * img_height, (row + 1) * img_height
            x_start, x_end = col * img_width, (col + 1) * img_width
            grid_frame[y_start:y_end, x_start:x_end] = img

        # Resize the grid frame if needed
        if resize:
            grid_frame = cv2.resize(grid_frame, (resize_width, resize_height))

        video_writer.write(grid_frame)

    video_writer.release()

def main():
    args = parse_arguments()

    # Load images
    image_sequences = load_images(args.data_dir, args.image_dirs)

    # Create video
    create_video(args.layout, image_sequences, args.output, args.fps, args.resize)
    print(f"Video saved to {args.output}")

if __name__ == "__main__":
    main()