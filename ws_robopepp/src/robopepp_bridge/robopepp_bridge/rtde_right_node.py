import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import ur_rtde.rtde_receive as rtde_receive

class RTDERightNode(Node):
    def __init__(self):
        super().__init__('rtde_right_node')
        self.declare_parameter('ip', '192.168.88.56')
        ip = self.get_parameter('ip').get_parameter_value().string_value
        try:
            self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        except Exception as e:
            self.get_logger().error(f'RTDE connection failed: {e}')
            self.rtde_r = None
        self.joint_pub = self.create_publisher(JointState, '/ur3_right/ground_truth_joint_states', 10)
        self.tcp_pub = self.create_publisher(PoseStamped, '/ur3_right/ground_truth_tcp', 10)
        self.timer = self.create_timer(0.02, self.update)

    def update(self):
        if self.rtde_r is None:
            return
        joints = self.rtde_r.getActualQ()
        pose = self.rtde_r.getActualTCPPose()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'right_shoulder_pan_joint',
            'right_shoulder_lift_joint',
            'right_elbow_joint',
            'right_wrist_1_joint',
            'right_wrist_2_joint',
            'right_wrist_3_joint'
        ]
        js.position = joints
        self.joint_pub.publish(js)
        tcp = PoseStamped()
        tcp.header = js.header
        tcp.pose.position.x = pose[0]
        tcp.pose.position.y = pose[1]
        tcp.pose.position.z = pose[2]
        self.tcp_pub.publish(tcp)


def main(args=None):
    rclpy.init(args=args)
    node = RTDERightNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
