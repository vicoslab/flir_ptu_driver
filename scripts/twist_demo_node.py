#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor

def deg2rad(deg):
    return deg * math.pi / 180.0

def rad2deg(rad):
    return rad * 180.0 / math.pi

class PTUTwistController(Node):
    def __init__(self):
        super().__init__('ptu_twist_controller')

        # Declare parameters with descriptions
        self.declare_parameter('pan_min_deg', -90.0, 
            ParameterDescriptor(description='Minimum pan angle in degrees'))
        self.declare_parameter('pan_max_deg', 90.0,
            ParameterDescriptor(description='Maximum pan angle in degrees'))
        self.declare_parameter('tilt_min_deg', -25.0,
            ParameterDescriptor(description='Minimum tilt angle in degrees'))
        self.declare_parameter('tilt_max_deg', 25.0,
            ParameterDescriptor(description='Maximum tilt angle in degrees'))
        self.declare_parameter('update_rate', 4.0,
            ParameterDescriptor(description='Rate to process twist commands in Hz'))

        # Get parameters and convert angle limits to radians
        self.pan_min = deg2rad(self.get_parameter('pan_min_deg').value)
        self.pan_max = deg2rad(self.get_parameter('pan_max_deg').value)
        self.tilt_min = deg2rad(self.get_parameter('tilt_min_deg').value)
        self.tilt_max = deg2rad(self.get_parameter('tilt_max_deg').value)
        self.update_rate = self.get_parameter('update_rate').value

        # Initialize current position state
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.last_twist = None

        # Create subscribers
        self.twist_sub = self.create_subscription(
            Twist,
            'ptu/twist_cmd_vel',
            self.twist_callback,
            10
        )
        self.state_sub = self.create_subscription(
            JointState,
            'ptu/state',
            self.state_callback,
            10
        )

        # Create publisher
        self.cmd_pub = self.create_publisher(
            JointState,
            'ptu/cmd',
            10
        )

        # Create timer for processing twist commands
        self.timer = self.create_timer(
            1.0/self.update_rate,
            self.update_position
        )

        self.get_logger().info('PTU Twist Controller initialized')

    def state_callback(self, msg):
        """Update current position from PTU state"""
        try:
            pan_idx = msg.name.index('ptu_pan')
            tilt_idx = msg.name.index('ptu_tilt')
            self.current_pan = msg.position[pan_idx]
            self.current_tilt = msg.position[tilt_idx]
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error processing state message: {str(e)}')

    def twist_callback(self, msg):
        """Store latest twist command"""
        self.last_twist = msg

    def clamp_position(self, pan, tilt):
        """Clamp pan and tilt values to min/max limits"""
        pan = max(min(pan, self.pan_max), self.pan_min)
        tilt = max(min(tilt, self.tilt_max), self.tilt_min)
        return pan, tilt

    def update_position(self):
        """Process latest twist command and update position"""
        if self.last_twist is None:
            return

        # Calculate new positions based on twist velocities
        # linear.x controls tilt, angular.z controls pan
        dt = 1.0 / self.update_rate
        new_pan = self.current_pan + (self.last_twist.angular.z * dt)
        new_tilt = self.current_tilt + (-self.last_twist.linear.x * dt)

        # Clamp to limits
        new_pan, new_tilt = self.clamp_position(new_pan, new_tilt)

        # Create command message
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = ['ptu_pan', 'ptu_tilt']
        cmd_msg.position = [new_pan, new_tilt]
        
        # Set velocities (absolute values of twist commands)
        cmd_msg.velocity = [abs(self.last_twist.angular.z/2), abs(self.last_twist.linear.x/2)]

        # Publish command
        self.cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PTUTwistController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()