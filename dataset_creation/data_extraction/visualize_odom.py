import numpy as np
import os
import matplotlib
from scipy.spatial.transform import Rotation as R

matplotlib.use("Agg")  # Set before importing pyplot
import matplotlib.pyplot as plt

DATA_DIR = "/workspace/Datasets/docking_3_sync_aligned"
# DATA_DIR = "/workspace/Datasets/flat_dataset"
RUN_NAME = "run1"

# From ROSBAG
baselink_TRANS_camera = np.array([0.35, 0.005, 0.135])
baselink_ROT_camera = np.array(
    [-0.46854314795176066, 0.46854306730111694, -0.5295921266402361, 0.5295912499203396]
)


def get_odom_file_paths(data_dir, run_name):
    odom_file_paths = []
    for file in os.listdir(os.path.join(data_dir, run_name)):
        if file.endswith("pose.txt"):
            odom_file_paths.append(os.path.join(data_dir, run_name, file))
    return odom_file_paths


def visualize_path(path, overlay_path=None):
    plt.scatter(path[:, 0], path[:, 1], c="r", label="Baselink Path", alpha=0.3, s=10)
    if overlay_path is not None:
        plt.scatter(
            overlay_path[:, 0],
            overlay_path[:, 1],
            c="b",
            label="Camera Path",
            alpha=0.7,
            s=2,
        )
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Path")
    plt.savefig("path.png")


if __name__ == "__main__":

    # Get the camera transformation
    baselink_T_camera = np.eye(4)
    baselink_T_camera[0:3, 3] = baselink_TRANS_camera
    baselink_T_camera[0:3, 0:3] = R.from_quat(baselink_ROT_camera).as_matrix()

    # Get all the odom file paths
    odom_file_paths = get_odom_file_paths(DATA_DIR, RUN_NAME)
    baselink_path = []
    camera_path = []
    for odom_file in odom_file_paths:
        odom_data = np.loadtxt(odom_file)
        camera_pose = odom_data @ baselink_T_camera
        baselink_path.append(odom_data[0:2, -1])
        camera_path.append(camera_pose[0:2, -1])
    baselink_path = np.array(baselink_path)
    camera_path = np.array(camera_path)

    # Visualize the paths
    visualize_path(path=baselink_path, overlay_path=camera_path)
