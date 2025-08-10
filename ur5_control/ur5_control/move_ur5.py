#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import rclpy
import tf2_ros
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from tf_transformations import quaternion_matrix, quaternion_from_matrix, quaternion_multiply
import rclpy.time

# Tunable parameters
Kp_pos = 2.0   # Proportional gain for position
Kp_ori = 1.0   # Proportional gain for orientation
position_tolerance = 0.01  # meters
orientation_tolerance = 0.01  # radians

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('PickAndPlace')

        # Publishers
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        # Service client for starting servoing
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/start_servo service...')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Start servoing
        self.start_servo()

        # Define target pose (in base_link frame)
        self.target_position = np.array([0.4, 0.2, 0.4])  # meters
        self.target_orientation = np.array([0.5, 0.5, 0.5, 0.5])  # quaternion (w last)

        # Timer for proportional control loop
        self.timer_period = 1.0 / 125  # 125 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def control_loop(self):
        try:
            # Get current EE pose
            trans = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())

            current_pos = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ])

            current_ori = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])

        except tf2_ros.LookupException:
            self.get_logger().warn("Transform not available yet.")
            return

        # Position error
        pos_error = self.target_position - current_pos
        pos_err_norm = np.linalg.norm(pos_error)

        # Orientation error (quaternion difference → axis-angle)
        q_error = quaternion_multiply(self.target_orientation,
                                      [ -current_ori[0], -current_ori[1], -current_ori[2], current_ori[3] ])
        angle = 2 * np.arccos(np.clip(q_error[3], -1.0, 1.0))
        if angle > np.pi:
            angle -= 2 * np.pi
        axis = q_error[:3]
        if np.linalg.norm(axis) > 1e-6:
            axis = axis / np.linalg.norm(axis)
        ori_error = axis * angle
        ori_err_norm = np.linalg.norm(ori_error)

        # Check if target reached
        if pos_err_norm < position_tolerance and ori_err_norm < orientation_tolerance:
            self.get_logger().info("Target pose reached.")
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(twist)  # publish zero twist
            self.get_logger().info(f"Pos_tol = {pos_error}")
            rclpy.shutdown()
            return

        # Proportional control
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = Kp_pos * pos_error[0]
        twist.twist.linear.y = Kp_pos * pos_error[1]
        twist.twist.linear.z = Kp_pos * pos_error[2]
        twist.twist.angular.x = Kp_ori * ori_error[0]
        twist.twist.angular.y = Kp_ori * ori_error[1]
        twist.twist.angular.z = Kp_ori * ori_error[2]

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
    pap_class = PickAndPlace()
    rclpy.spin(pap_class)
    pap_class.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
