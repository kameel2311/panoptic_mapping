import rosbag2_py
from sensor_msgs.msg import CameraInfo
import sys
import cv2
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DataExtractor:
    def __init__(
        self,
        input_bag_path,
        input_topic,
        rgb_intrinsics_topic,
        modified_intrinsics,
    ):
        self.bridge = CvBridge()
        self.input_bag_path = input_bag_path
        self.input_topic = input_topic
        self.rgb_intrinsics_topic = rgb_intrinsics_topic
        self.modified_intrinsics = modified_intrinsics
        self.converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
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

    def modify_and_write_bag(self):
        # Get Intristics
        rgb_intrinsics = self.get_intrisnics(
            self.rgb_intrinsics_topic, modified=self.modified_intrinsics
        )
        print("RGB Intrinsics: ", rgb_intrinsics)

        # Loop through the depth messages and align them
        reader = self.create_reader()

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
                if topic == self.input_topic:
                    # Deserialize the message
                    data = deserialize_message(data, Image)

                    # Convert ROS Image message to OpenCV
                    depth_image = self.bridge.imgmsg_to_cv2(
                        data, desired_encoding="passthrough"
                    )
                    depth_image = np.asarray(depth_image, dtype=np.float32)

            # reader.close()
            print("New bag with added topic saved successfully.")

        finally:
            del reader


def main():
    input_bag_path = "/workspace/Datasets/ROS2_bags/docking_3/docking_3.db3"
    input_topic = "/depth_image"
    rgb_intrinsics_topic = "/color_intrinsics"
    modified_intrinsics = False

    modifier = DataExtractor(
        input_bag_path,
        input_topic,
        rgb_intrinsics_topic,
        modified_intrinsics,
    )
    modifier.modify_and_write_bag()


if __name__ == "__main__":
    main()
