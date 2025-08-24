import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np

class RoboPEPPNode(Node):
    """Skeleton node demonstrating GSAM-2 -> RoboPEPP pipeline."""
    def __init__(self):
        super().__init__('robopepp_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.left_joint_pub = self.create_publisher(JointState, '/ur3_left/estimated_joint_states', 10)
        self.right_joint_pub = self.create_publisher(JointState, '/ur3_right/estimated_joint_states', 10)
        self.left_tcp_pub = self.create_publisher(PoseStamped, '/ur3_left/estimated_tcp', 10)
        self.right_tcp_pub = self.create_publisher(PoseStamped, '/ur3_right/estimated_tcp', 10)
        self.debug_pub = self.create_publisher(Image, '/robopepp/debug_image', 10)
        self.get_logger().info('RoboPEPP node initialized (skeleton).')

    def image_callback(self, msg: Image):
        """Placeholder callback that republishes the incoming image."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # TODO: Integrate GSAM-2, RoboPEPP inference, FK, and PnP here.
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoboPEPPNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
