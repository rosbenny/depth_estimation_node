import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Image
import math
import numpy as np

try:
    from cv_bridge import CvBridge
    cv_available = True
except ImportError:
    cv_available = False


class DepthEstimator(Node):
    def __init__(self):
        super().__init__('depth_estimator')

        # Subscriber for X, Y input
        self.subscription_xy = self.create_subscription(
            Float32MultiArray,
            '/xy_input',
            self.xy_callback,
            10)

        # Subscriber for camera depth image (if available)
        if cv_available:
            self.subscription_camera = self.create_subscription(
                Image,
                '/camera/depth/image_raw',
                self.camera_callback,
                10)
            self.bridge = CvBridge()

        # Publisher for depth output
        self.publisher_ = self.create_publisher(Float32, '/depth_camera_distance', 10)

        self.get_logger().info('Depth Estimator Node Started')

    def xy_callback(self, msg):
        if len(msg.data) >= 2:
            x = msg.data[0]
            y = msg.data[1]
            distance = math.sqrt(x**2 + y**2)
            self.publish_distance(distance, source="XY")
        else:
            self.get_logger().warn("Received XY message with less than 2 values")

    def camera_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_array = np.array(depth_image, dtype=np.float32)

            # Get image dimensions
            height, width = depth_array.shape

            # Get center pixel value
            center_x = width // 2
            center_y = height // 2
            center_depth = depth_array[center_y, center_x]

            if np.isnan(center_depth) or center_depth <= 0.0:
                self.get_logger().warn("⚠️ Center pixel has no valid depth")
                return

            self.publish_distance(center_depth, source="Camera-Center")

        except Exception as e:
            self.get_logger().error(f"Failed to process camera data: {e}")

    def publish_distance(self, distance, source):
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)

        if distance < 1.0:
            self.get_logger().warn(f'⚠️ [{source}] Obstacle too close! Distance: {distance:.2f} m')
        else:
            self.get_logger().info(f'✅ [{source}] Safe distance: {distance:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

