#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class PositionSubscriberNode(Node):
    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'position_difference',
            self.position_callback,
            10)
        self.subscription  # prevent unused variable warning

    def position_callback(self, msg):
        self.get_logger().info(f'Received object position: x={msg.x:.2f}, y={msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    position_subscriber = PositionSubscriberNode()
    try:
        rclpy.spin(position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        position_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()