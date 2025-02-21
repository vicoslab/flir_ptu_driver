#!/usr/bin/env python3

import math
import time
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState
from ptu_driver import PTU
from threading import Lock

def deg2rad(deg):
    return deg * math.pi / 180.0

def rad2deg(rad):
    return rad * 180.0 / math.pi

class PTUNode(Node):
    def __init__(self):
        super().__init__('ptu_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('publishing_rate', 5.0)  # Hz

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.rate = self.get_parameter('publishing_rate').value
        
        # Initialize PTU
        try:
            self.ptu = PTU(port=port, baud=baud)
            self.get_logger().info(f'Connected to PTU on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to PTU: {str(e)}')
            raise e

        # Store conversion factors for speed calculations
        # Convert from rad/s to positions/sec
        self.pan_speed_conv = abs(self.ptu.panAngleToPosition(rad2deg(1.0)))
        self.tilt_speed_conv = abs(self.ptu.tiltAngleToPosition(rad2deg(1.0)))

        # Create publishers and subscribers
        self.state_pub = self.create_publisher(
            JointState,
            'ptu/state',
            10
        )
        self.cmd_sub = self.create_subscription(
            JointState,
            'ptu/cmd',
            self.cmd_callback,
            10
        )

        # Create mutex for thread safety
        self.ptu_lock = Lock()

        # Create timer for publishing state
        self.timer = self.create_timer(
            1.0/self.rate,
            self.publish_state
        )

        self.get_logger().info('PTU node initialized')

    def publish_state(self):
        """Publish current PTU state as JointState message"""
        try:
            with self.ptu_lock:
                pan_pos, tilt_pos = self.ptu.getPosition()

            # Convert positions to angles (radians)
            pan_angle_deg = pan_pos * (self.ptu.panResolution / 3600.0)
            tilt_angle_deg = tilt_pos * (self.ptu.tiltResolution / 3600.0)
            
            pan_angle_rad = deg2rad(pan_angle_deg)
            tilt_angle_rad = deg2rad(tilt_angle_deg)

            # Create and publish JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['ptu_pan', 'ptu_tilt']
            msg.position = [pan_angle_rad, tilt_angle_rad]
            self.state_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing state: {str(e)}')

    def cmd_callback(self, msg):
        """Handle incoming command messages"""
        try:
            # Validate message
            if len(msg.position) != 2 or len(msg.name) != 2:
                self.get_logger().warning('Invalid command message format')
                return

            # Extract commanded positions and velocities
            pan_angle_rad = None
            tilt_angle_rad = None
            pan_velocity_rad = None
            tilt_velocity_rad = None

            for i, name in enumerate(msg.name):
                if name == 'ptu_pan':
                    pan_angle_rad = msg.position[i]
                    pan_velocity_rad = msg.velocity[i] if msg.velocity else None
                elif name == 'ptu_tilt':
                    tilt_angle_rad = msg.position[i]
                    tilt_velocity_rad = msg.velocity[i] if msg.velocity else None

            if None in [pan_angle_rad, tilt_angle_rad]:
                self.get_logger().warning('Missing pan or tilt values in command')
                return

            # Convert angles to positions
            pan_angle_deg = rad2deg(pan_angle_rad)
            tilt_angle_deg = rad2deg(tilt_angle_rad)
            pan_pos = self.ptu.panAngleToPosition(pan_angle_deg)
            tilt_pos = self.ptu.tiltAngleToPosition(tilt_angle_deg)

            # Convert velocities if provided
            if pan_velocity_rad is not None and tilt_velocity_rad is not None:
                # Convert from rad/s to positions/sec
                pan_speed = abs(int(pan_velocity_rad * self.pan_speed_conv))
                tilt_speed = abs(int(tilt_velocity_rad * self.tilt_speed_conv))
                
                # Send command to PTU with speed
                with self.ptu_lock:
                    self.ptu.setPositionAndSpeed(pan_pos, tilt_pos, pan_speed, tilt_speed)
            else:
                # Send position command only
                with self.ptu_lock:
                    self.ptu.setPosition(pan_pos, tilt_pos)

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def cleanup(self):
        """Clean up PTU connection"""
        if hasattr(self, 'ptu'):
            self.ptu.halt()
            del self.ptu

def main(args=None):
    rclpy.init(args=args)
    node = PTUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()