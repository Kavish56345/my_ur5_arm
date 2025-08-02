#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Team ID:          LB#1708
# Author List:		Kavish V, Jaishankar A, Madhuranthakam Tarusi
# Filename:		    task2a.py

##################################################

import rclpy
import sys
import rclpy
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState

linear_speed_limit = 0.1
angular_speed_limit = 0.1
orientation_tolerance = 0.01
position_tolerance = 0.05

boxes = {}
ebot = {}

class PickAndPlace(Node):

    def __init__(self):

        super().__init__('PickAndPlace')                                          

        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_jog_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/start_servo service...')
        
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.target_joint_positions = np.array([0.0, -2.3911, 2.40855, -1.5708, -1.5708, 3.14159])  # Adjust as needed
        self.current_joint_positions = None
        self.tolerance = 0.02  

        self.start_servo()
        # self.jog_timer = self.create_timer(0.1, self.publish_joint_jog)

        self.timer_period = 1.0 / 125  # 125 Hz
        self.side_duration = 2.0  # seconds per side
        self.sides = [
            {'y':  0.2, 'z':  0.0},  # +Y
            {'y':  0.0, 'z':  0.2},  # +Z
            {'y': -0.2, 'z':  0.0},  # -Y
            {'y':  0.0, 'z': -0.2},  # -Z
        ]
        self.current_side = 0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.start_time

        if elapsed > self.side_duration:
            self.current_side += 1
            if self.current_side >= len(self.sides):
                self.get_logger().info("Finished drawing square.")
                rclpy.shutdown()
                return
            self.start_time = now

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = self.sides[self.current_side]['y']
        twist.twist.linear.z = self.sides[self.current_side]['z']
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.publisher.publish(twist)

    def joint_state_callback(self, msg):
        self.current_joint_positions = np.array(msg.position)
        
    def publish_joint_jog(self):
        if self.current_joint_positions is None:
            return

        position_error = self.target_joint_positions - self.current_joint_positions

        if np.all(np.abs(position_error) < self.tolerance):
            jog_msg = JointJog()
            jog_msg.header.stamp = self.get_clock().now().to_msg()
            
            jog_msg.joint_names = [ "shoulder_pan_joint",
                                    "shoulder_lift_joint",
                                    "elbow_joint",
                                    "wrist_1_joint",
                                    "wrist_2_joint",
                                    "wrist_3_joint",    ]
            
            jog_msg.velocities = [0.0 for _ in range(6)]
            jog_msg.duration = 0.1

            self.joint_jog_pub.publish(jog_msg)
            self.get_logger().info('Target position reached.')
            self.jog_timer.cancel()
            self.flag = True
            return 

        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()
        
        jog_msg.joint_names = [ "shoulder_pan_joint",
                                    "shoulder_lift_joint",
                                    "elbow_joint",
                                    "wrist_1_joint",
                                    "wrist_2_joint",
                                    "wrist_3_joint",    ]
        
        jog_msg.velocities = (10.0 * position_error).tolist()
        jog_msg.duration = 0.1

        self.joint_jog_pub.publish(jog_msg)
        self.get_logger().info(f'Published jog command: {jog_msg.velocities}')

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
    node = rclpy.create_node('PickAndPlace')                    
    node.get_logger().info('Node created: Pick And Place')        

    pap_class = PickAndPlace()                                     
    rclpy.spin(pap_class)                                      

    pap_class.destroy_node()                                   

    rclpy.shutdown()                                                

if __name__ == '__main__':

    main()