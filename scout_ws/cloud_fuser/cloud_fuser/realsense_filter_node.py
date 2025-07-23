import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import numpy as np

class RealSensePointCloudFilter(Node):
    def __init__(self):
        super().__init__('realsense_filter_node')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10)

        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_depth_cloud',
            10)

    def pointcloud_callback(self, msg):
        # Read points and filter
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        filtered = []

        for p in gen:
            x, y, z = p
            if 0.2 < z < 2.5 and 0.0 < y < 2.5:
                filtered.append([x, y, z])

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        filtered_cloud = pc2.create_cloud_xyz32(header, filtered)
        self.publisher.publish(filtered_cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

