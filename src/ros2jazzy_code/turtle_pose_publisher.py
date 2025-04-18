import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import socket
import struct

class TurtlePosePublisher(Node):
    """
    A ROS 2 node that listens to turtle1/pose and sends the pose as raw UDP
    to an external application (e.g., NS-3).
    """

    def __init__(self):
        super().__init__('turtle_pose_publisher')

        # Subscribe to turtle1's pose topic
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_listener,
            10
        )

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ns3_address = ('10.1.1.2', 12345)

        self.get_logger().info(f"UDP socket initialized to send to {self.ns3_address}")

    def pose_listener(self, msg: Pose):
        """Callback for every new turtle pose."""
        x, y, theta = msg.x, msg.y, msg.theta
        packed_data = struct.pack('fff', x, y, theta)

        try:
            self.sock.sendto(packed_data, self.ns3_address)
            self.get_logger().info(f"Sent UDP Data: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        except socket.error as e:
            self.get_logger().error(f"Socket error while sending data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down TurtlePosePublisher.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
