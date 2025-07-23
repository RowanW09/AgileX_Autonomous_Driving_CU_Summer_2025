import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2sm
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudFuser(Node):
    def __init__(self):
        super().__init__('cloud_fuser_V1')
        
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 0.4)
        self.declare_parameter('voxel_size', 0.05)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_rgbd = Subscriber(self, PointCloud2, '/camera/camera/depth/color/points') #'/filtered_depth_cloud') #'/camera/camera/depth/color/points')
        self.sub_livox = Subscriber(self, PointCloud2, '/livox/lidar')

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgbd, self.sub_livox], queue_size=50, slop=0.5)
        self.sync.registerCallback(self.callback)

        self.pub = self.create_publisher(PointCloud2, '/fused_points', 10)
    
    # custom cloud chain filter
    def passthrough_z(self, points, z_min=-0.1, z_max=0.4):
    	return [pt for pt in points if z_min <= pt[2] <= z_max]
    	
    def voxel_filter(self, points, voxel_size=0.05):
        rounded = np.round(np.array(points) / voxel_size) * voxel_size
        unique = np.unique(rounded, axis=0)
        return unique.tolist()
 
    def callback(self, cloud_rgbd, cloud_livox):
        try:
            transform_rgbd = self.tf_buffer.lookup_transform(
                'base_link', cloud_rgbd.header.frame_id, rclpy.time.Time())
            transform_livox = self.tf_buffer.lookup_transform(
                'base_link', cloud_livox.header.frame_id, rclpy.time.Time())

            cloud_rgbd_trans = tf2sm.do_transform_cloud(cloud_rgbd, transform_rgbd)
            cloud_livox_trans = tf2sm.do_transform_cloud(cloud_livox, transform_livox)

            # Convert to iterable formats
            pts_rgbd = list(point_cloud2.read_points(cloud_rgbd_trans, skip_nans=True))
            pts_livox = list(point_cloud2.read_points(cloud_livox_trans, skip_nans=True))
            
            z_min = self.get_parameter('z_min').value
            z_max = self.get_parameter('z_max').value
            voxel_size = self.get_parameter('voxel_size').value

	    # Filter both point sets
            pts_rgbd = self.passthrough_z(pts_rgbd, z_min, z_max)
            pts_livox = self.passthrough_z(pts_livox, z_min, z_max)
	    
	    # voxel grid filter
            xyz_only = [(x, y, z) for x, y, z, *_ in pts_rgbd + pts_livox]
            pts_fused = self.voxel_filter(xyz_only, voxel_size)


            #pts_fused = pts_rgbd + pts_livox

            header = cloud_rgbd.header
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'

            fused_cloud = point_cloud2.create_cloud_xyz32(header, [(x, y, z) for x, y, z, *_ in pts_fused])

            self.pub.publish(fused_cloud)

        except Exception as e:
            self.get_logger().warn(f"Fusion failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFuser()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

