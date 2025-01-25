import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile


class SyncRepublishNode(Node):
    def __init__(self):
        super().__init__("sync_republish_node")

        # QoS for reliability
        qos_profile = QoSProfile(depth=10)

        # Subscribers with message_filters
        self.rgb_sub = Subscriber(self, Image, "/rgb_image", qos_profile=qos_profile)
        self.depth_sub = Subscriber(
            self, Image, "/depth_image", qos_profile=qos_profile
        )
        self.odom_sub = Subscriber(self, Odometry, "/odom", qos_profile=qos_profile)

        # Approximate time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.odom_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.callback)

        # Publishers
        self.rgb_pub = self.create_publisher(Image, "/rgb_synced", qos_profile)
        self.depth_pub = self.create_publisher(Image, "/depth_synced", qos_profile)
        self.odom_pub = self.create_publisher(Odometry, "/odom_synced", qos_profile)

        self.get_logger().info("SyncRepublishNode is up and running.")

    def callback(self, rgb_msg, depth_msg, odom_msg):
        # Log synchronized message receipt
        self.get_logger().info("Synchronized messages received")

        # Uniy the headers
        unified_header = rgb_msg.header.stamp
        depth_msg.header.stamp = unified_header
        odom_msg.header.stamp = unified_header

        # Republish the synchronized messages
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncRepublishNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
