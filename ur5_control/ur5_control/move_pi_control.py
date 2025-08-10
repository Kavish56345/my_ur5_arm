#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
import sys
import tf2_ros
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
import rclpy.time

linear_speed_limit = 0.5  # m/s max
position_tolerance = 0.01  # m
Kp = 10.0  # proportional gain (scalar for all axes)
Ki = 0.1  # integral gain (scalar for all axes)

class PIPositionControl(Node):
    def __init__(self):
        super().__init__('pi_position_control')

        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/start_servo service...')

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Goal position in base frame (m)
        self.goal_position = np.array([0.4, 0.2, 0.4])

        # Integral term
        self.integral_error = np.zeros(3)

        self.start_servo()

        self.timer = self.create_timer(1.0 / 125, self.timer_callback)

    def timer_callback(self):
        try:
            # Lookup current EE pose in base frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            )
        except:
            return

        # Current position
        p = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])

        # Position error
        error = self.goal_position - p

        # Check if within tolerance
        if np.linalg.norm(error) < position_tolerance:
            self.get_logger().info('Goal position reached.')
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            self.publisher.publish(twist)
            self.get_logger().info(f"Pos_tol = {np.linalg.norm(error)}")
            # rclpy.shutdown()
            return

        # Integrate error
        self.integral_error += error * (1.0 / 125)  # dt = 1/125s

        # PI control output
        cmd_linear = Kp * error + Ki * self.integral_error

        # Limit speed
        cmd_linear = np.clip(cmd_linear, -linear_speed_limit, linear_speed_limit)

        # Publish twist (only linear part, orientation control = 0)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = cmd_linear[0]
        twist.twist.linear.y = cmd_linear[1]
        twist.twist.linear.z = cmd_linear[2]
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.publisher.publish(twist)

    def start_servo(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Servoing started successfully.')
        else:
            self.get_logger().error('Failed to start servoing.')

def main():
    rclpy.init(args=sys.argv)
    node = PIPositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
