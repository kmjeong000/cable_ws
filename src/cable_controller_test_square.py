#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class MoveEndSphere(Node):
    """
    Move the end sphere to a sequence of target positions
    by publishing geometry_msgs/Point on 'cable_target_position'.
    """
    
    def __init__(self):
        super().__init__('move_end_sphere')

        # Publisher for target positions
        self.publisher_ = self.create_publisher(
            Point,
            'cable_target_position',
            10 # queue size
        )

        # Define the target positions
        self.targets = [
            (0.2, 0.6, 2.0),
            (0.2, 0.3, 2.0),
            (-0.2, 0.3, 2.0),
            (-0.2, 0.6, 2.0),
        ]
        self.current_index = 0

        # Timer to publish at fixed interval (2sec)
        self.timer_period = 2.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('MoveEndSphere node started. Publishing target positions every 2 seconds.')

    def timer_callback(self):
        # Select current target
        x, y, z = self.targets[self.current_index]

        # Create and publish Point message
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing target position #{self.current_index + 1}: '
            f'[{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]'
        )

        # Move to next target (loop back to start)
        self.current_index = (self.current_index + 1) % len(self.targets)

def main(args=None):
    rclpy.init(args=args)

    node = MoveEndSphere()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()