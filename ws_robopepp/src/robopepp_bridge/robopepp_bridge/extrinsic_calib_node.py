import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

class ExtrinsicCalibNode(Node):
    def __init__(self):
        super().__init__('extrinsic_calib_node')
        self.declare_parameter('output', 'config/extrinsics.yaml')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.output_path = self.get_parameter('output').get_parameter_value().string_value
        self.get_logger().info('Extrinsic calibration node ready. Press q to save transform.')

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('calibration', cv_image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            # Placeholder: identity transform
            T = {
                'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
            with open(self.output_path, 'w') as f:
                yaml.safe_dump(T, f)
            self.get_logger().info(f'Wrote extrinsics to {self.output_path}')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ExtrinsicCalibNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
