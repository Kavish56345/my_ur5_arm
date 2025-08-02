#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class DrawSquare(Node):

    def __init__(self):
        super().__init__('draw_square')
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        time.sleep(1)

    def send_twist(self, dx=0.0, dy=0.0, dz=0.0, duration=1.0):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = dx
        twist.twist.linear.y = dy
        twist.twist.linear.z = dz

        rate = self.create_rate(10)  # 10 Hz
        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < duration:
            self.twist_pub.publish(twist)
            rate.sleep()

        # Stop after move
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        self.twist_pub.publish(stop)

    def draw_standing_square(self, side_length=0.1, speed=0.05):
        duration = side_length / speed

        # Move Up
        self.send_twist(dz=speed, duration=duration)
        # Move Right
        self.send_twist(dx=speed, duration=duration)
        # Move Down
        self.send_twist(dz=-speed, duration=duration)
        # Move Left
        self.send_twist(dx=-speed, duration=duration)

def main(args=None):
    rclpy.init(args=args)
    node = DrawSquare()
    node.draw_standing_square()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
