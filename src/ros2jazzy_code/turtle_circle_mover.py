import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
class TurtleCircleMover(Node):
    def __init__(self):
        super().__init__('turtle_circle_mover')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.move_turtle)
        self.linear_velocity = 1.0  # Adjust as needed
        self.angular_velocity = 0.5 # Adjust as needed
    def move_turtle(self):
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')
def main(args=None):
    rclpy.init(args=args)
    turtle_circle_mover = TurtleCircleMover()
    rclpy.spin(turtle_circle_mover)
    turtle_circle_mover.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
