import numpy as np
import os
import matplotlib
import cv2
import matplotlib.pyplot as plt

# DATA_DIR = "/workspace/Datasets/docking_3_sync_aligned"
# DATA_DIR = "/workspace/Datasets/flat_dataset"
DATA_DIR = "/workspace/Datasets/simulated_data"
RUN_NAME = "run1"


def get_depth_file_paths(data_dir, run_name):
    depth_file_paths = []
    for file in os.listdir(os.path.join(data_dir, run_name)):
        if file.endswith("depth.tiff"):
            depth_file_paths.append(os.path.join(data_dir, run_name, file))
    depth_file_paths.sort()
    return depth_file_paths


if __name__ == "__main__":

    # Get all the depth file paths
    depth_file_paths = get_depth_file_paths(DATA_DIR, RUN_NAME)
    print(f"Found {len(depth_file_paths)} depth files.")

    for depth_file in depth_file_paths:
        depth_data = cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH)
        # depth_data = cv2.bilateralFilter(depth_data, d=25, sigmaColor=1, sigmaSpace=75)
        fig, ax = plt.subplots(2, 1, figsize=(10, 8))  # 2 rows, 1 column
        # depth_data = cv2.GaussianBlur(depth_data, (9, 9), 0)q
        ax[0].imshow(depth_data, cmap="gray")
        ax[0].set_title("Depth Image")
        ax[0].axis("off")  # Hide axis for a cleaner look

        # Bottom subplot: Histogram of Depth Values
        depth_values = depth_data.flatten()  # Flatten to 1D array
        depth_values = depth_values[depth_values > 0]  # Remove zero values
        ax[1].hist(depth_values, bins=100, color="blue", alpha=0.7)
        ax[1].set_title("Histogram of Depth Values")
        ax[1].set_xlabel("Depth Value")
        ax[1].set_ylabel("Frequency")
        ax[1].grid(True)

        # Show the figure
        plt.tight_layout()  # Adjust layout to prevent overlap
        plt.show()
