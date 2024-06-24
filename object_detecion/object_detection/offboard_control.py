import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.move_forward)
        self.start_time = self.get_clock().now()
        self.velocity = 0.5  # 0.5 m/s
        self.distance_interval = 1.0  # Print position every 1 meter
        self.current_position = None
        self.start_position = None
        self.total_distance = 0.0

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.get_logger().info(f"Current position: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}, z={self.current_position.z:.2f}")

    def move_forward(self):
        if self.current_position is None:
            return

        # Calculate distance traveled since last position log
        if self.start_position is None:
            self.start_position = self.current_position

        distance_traveled = math.sqrt(
            (self.current_position.x - self.start_position.x) ** 2 +
            (self.current_position.y - self.start_position.y) ** 2 +
            (self.current_position.z - self.start_position.z) ** 2
        )

        if distance_traveled >= self.distance_interval:
            self.get_logger().info(f"Distance traveled: {self.total_distance + distance_traveled:.2f} meters")
            self.start_position = self.current_position
            self.total_distance += distance_traveled

        # Check if total distance reached
        if self.total_distance >= 3.0:
            self.timer.cancel()
            msg = Twist()
            msg.linear.x = 0.0
            self.publisher.publish(msg)
            self.get_logger().info("Movement completed")
            rclpy.shutdown()

        # Publish movement command
        msg = Twist()
        msg.linear.x = self.velocity
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()
    rclpy.spin(robot_mover)

if __name__ == '__main__':
    main()



