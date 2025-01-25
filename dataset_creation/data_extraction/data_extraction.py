import rosbag2_py
from sensor_msgs.msg import CameraInfo
import sys
import cv2
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import os
from scipy.spatial.transform import Rotation as R
from PIL import Image as PILImage


class DataExtractor:
    def __init__(
        self,
        input_bag_path,
        rgb_topic,
        depth_topic,
        pose_topic,
        intrinsics_topic,
        modified_intrinsics,
        output_dir,
        run_folder_name,
        pose_message_type=Odometry,
    ):
        self.bridge = CvBridge()
        self.input_bag_path = input_bag_path
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.pose_topic = pose_topic
        self.pose_message_type = pose_message_type
        self.intrinsics_topic = intrinsics_topic
        self.intrinsics = None
        self.modified_intrinsics = modified_intrinsics
        self.output_dir = output_dir
        self.run_folder_name = run_folder_name

        self.converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )

        if pose_message_type is not Odometry:
            raise ValueError(
                f"Only Odometry is currently supported, please add support for {pose_message_type}."
            )

        # Create the output directories
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"Created new output directory at '{self.output_dir}'.")
        else:
            raise ValueError(
                "Output directory already exists. Please provide a new one."
            )
        if not os.path.isdir(os.path.join(self.output_dir, self.run_folder_name)):
            os.makedirs(os.path.join(self.output_dir, self.run_folder_name))
            print(
                f"Created new output directory at '{os.path.join(self.output_dir, self.run_folder_name)}'."
            )
        else:
            raise ValueError(
                "Output directory already exists. Please provide a new one."
            )

    def create_reader(self):
        reader = rosbag2_py.SequentialReader()
        storage_options_in = rosbag2_py.StorageOptions(
            uri=self.input_bag_path, storage_id="sqlite3"
        )
        reader.open(storage_options_in, self.converter_options)
        return reader

    def get_intrisnics(self, intrinsics_topic, modified=False):
        self.tmp_reader = self.create_reader()
        try:
            while self.tmp_reader.has_next():
                topic, data, timestamp = self.tmp_reader.read_next()
                print("Topic: ", topic)
                print("Modified Topic ?: ", modified)
                if topic == intrinsics_topic:
                    data = deserialize_message(data, CameraInfo)
                    if modified:
                        center = np.array([data.k[2], data.k[5]], dtype=np.float32)
                    else:
                        center = np.array([data.k[5], data.k[2]], dtype=np.float32)
                    intrinsics_data = {
                        "focal_length": np.array(
                            [data.k[0], data.k[4]], dtype=np.float32
                        ),
                        "center": center,
                        "dimensions": (data.width, data.height),
                    }
                    return intrinsics_data
        finally:
            del self.tmp_reader

    def save_extriniscs(self, file_name="Intrinsics.txt"):
        if self.intrinsics is None:
            raise ValueError("Intrinsics not found. Run get_intrinsics() first.")
        # Check if file exists
        file_path = os.path.join(self.output_dir, file_name)
        # Write intrinsics to file
        if not os.path.exists(file_path):
            with open(file_path, "w") as f:
                data = [
                    f"Res_x: {self.intrinsics['dimensions'][0]}\n",
                    f"Res_y: {self.intrinsics['dimensions'][1]}\n",
                    f"u: {self.intrinsics['center'][0]}\n",
                    f"v: {self.intrinsics['center'][1]}\n",
                    f"f_x: {self.intrinsics['focal_length'][0]}\n",
                    f"f_y: {self.intrinsics['focal_length'][1]}\n",
                ]
                f.writelines(data)

    def extarct_from_bag(self):
        # Get Intristics
        if self.intrinsics_topic is not None:
            self.intrinsics = self.get_intrisnics(
                self.intrinsics_topic, modified=self.modified_intrinsics
            )
            print("Extarcted Intrinsics: ", self.intrinsics)
            # Save Intrinsics
            self.save_extriniscs()

        # Loop through the depth messages and align them
        reader = self.create_reader()

        # Bookeeping the number of frames with timestamp
        img_no_and_ts = {}
        rgb_image_no = 0
        depth_image_no = 0
        pose_no = 0

        # Go through the bag file
        try:  # As closing methods not implemented all distros of rosbag2_py
            # Get topics and types from the input bag and create them in the output bag
            metadata = reader.get_all_topics_and_types()
            for topic_metadata in metadata:
                print(topic_metadata.name, topic_metadata.type)

            # Read messages from the input bag and write them to the output bag
            print("Starting to read and write messages...")
            while reader.has_next():
                topic, data, timestamp = reader.read_next()

                # If the message is from the input topic, generate the new topic's message
                if topic == self.depth_topic:
                    # Deserialize the message
                    data = deserialize_message(data, Image)
                    # Convert ROS Image message to OpenCV, depth the script way PIL can be ommited
                    depth_image = PILImage.fromarray(
                        self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                    )
                    # Save the image
                    image_id = "%06d" % depth_image_no
                    depth_image_no += 1
                    if image_id not in img_no_and_ts:
                        img_no_and_ts[image_id] = data.header.stamp
                    else:
                        assert (
                            img_no_and_ts[image_id] == data.header.stamp
                        ), f"Image timestamps don't match for frame {image_id}"
                    depth_image.save(
                        os.path.join(
                            self.output_dir,
                            self.run_folder_name,
                            image_id + "_depth.tiff",
                        )
                    )
                elif topic == self.rgb_topic:
                    # Deserialize the message
                    data = deserialize_message(data, Image)
                    # Convert ROS Image message to OpenCV
                    rgb_image = self.bridge.imgmsg_to_cv2(
                        data, desired_encoding="passthrough"
                    )
                    # Save the image
                    image_id = "%06d" % rgb_image_no
                    rgb_image_no += 1
                    if image_id not in img_no_and_ts:
                        img_no_and_ts[image_id] = data.header.stamp
                    else:
                        assert (
                            img_no_and_ts[image_id] == data.header.stamp
                        ), f"Image timestamps don't match for frame {image_id}"
                    cv2.imwrite(
                        os.path.join(
                            self.output_dir,
                            self.run_folder_name,
                            image_id + "_color.png",
                        ),
                        cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB),
                    )
                elif topic == self.pose_topic:
                    # Deserialize the message
                    data = deserialize_message(data, self.pose_message_type)
                    # THIS IS ONLY IF IT IS ODOMETRY
                    position = data.pose.pose.position
                    orientation = data.pose.pose.orientation
                    orientation_matrix = R.from_quat(
                        [orientation.x, orientation.y, orientation.z, orientation.w]
                    ).as_matrix()
                    # Save the pose
                    image_id = "%06d" % pose_no
                    pose_no += 1
                    if image_id not in img_no_and_ts:
                        img_no_and_ts[image_id] = data.header.stamp
                    else:
                        assert (
                            img_no_and_ts[image_id] == data.header.stamp
                        ), f"Image timestamps don't match for frame {image_id}"
                    with open(
                        os.path.join(
                            self.output_dir,
                            self.run_folder_name,
                            image_id + "_pose.txt",
                        ),
                        "w",
                    ) as f:
                        f.write(
                            f"{orientation_matrix[0][0]} {orientation_matrix[0][1]} {orientation_matrix[0][2]} {position.x}\n"
                        )
                        f.write(
                            f"{orientation_matrix[1][0]} {orientation_matrix[1][1]} {orientation_matrix[1][2]} {position.y}\n"
                        )
                        f.write(
                            f"{orientation_matrix[2][0]} {orientation_matrix[2][1]} {orientation_matrix[2][2]} {position.z}\n"
                        )
                        f.write("0.0 0.0 0.0 1.0")

            # reader.close()
            print("Data fully exatracted from ROS Bag.")

        finally:
            del reader


def main():
    input_bag_path = (
        "/workspace/Datasets/ROS2_bags/docking_3_synced/docking_3_synced_0.db3"
    )
    output_dir = "/workspace/Datasets/docking_3_synced/"
    run_folder_name = "run1"
    modified_intrinsics = False

    modifier = DataExtractor(
        input_bag_path=input_bag_path,
        rgb_topic="/rgb_synced",
        depth_topic="/depth_synced",
        pose_topic="/odom_synced",
        intrinsics_topic="/color_intrinsics",
        modified_intrinsics=modified_intrinsics,
        output_dir=output_dir,
        run_folder_name=run_folder_name,
        pose_message_type=Odometry,
    )

    modifier.extarct_from_bag()


if __name__ == "__main__":
    main()
