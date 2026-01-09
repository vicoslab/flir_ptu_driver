#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

def deg2rad(deg):
    return deg * math.pi / 180.0

def rad2deg(rad):
    return rad * 180.0 / math.pi


class PTUTwistController:
    def __init__(self):
        rospy.init_node('ptu_twist_controller')

        # Parameters (degrees, converted to radians)
        pan_min_deg = rospy.get_param('~pan_min_deg', -90.0)
        pan_max_deg = rospy.get_param('~pan_max_deg', 90.0)
        tilt_min_deg = rospy.get_param('~tilt_min_deg', -25.0)
        tilt_max_deg = rospy.get_param('~tilt_max_deg', 25.0)
        self.update_rate = rospy.get_param('~update_rate', 4.0)

        self.pan_min = deg2rad(pan_min_deg)
        self.pan_max = deg2rad(pan_max_deg)
        self.tilt_min = deg2rad(tilt_min_deg)
        self.tilt_max = deg2rad(tilt_max_deg)

        # Current state
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.last_twist = None

        # Subscribers
        self.twist_sub = rospy.Subscriber(
            'ptu/twist_cmd_vel',
            Twist,
            self.twist_callback,
            queue_size=10
        )

        self.state_sub = rospy.Subscriber(
            'ptu/state',
            JointState,
            self.state_callback,
            queue_size=10
        )

        # Publisher
        self.cmd_pub = rospy.Publisher(
            'ptu/cmd',
            JointState,
            queue_size=10
        )

        # Timer
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.update_rate),
            self.update_position
        )

        rospy.loginfo('PTU Twist Controller initialized')

    def state_callback(self, msg):
        """Update current position from PTU state"""
        try:
            pan_idx = msg.name.index('ptu_pan')
            tilt_idx = msg.name.index('ptu_tilt')
            self.current_pan = msg.position[pan_idx]
            self.current_tilt = msg.position[tilt_idx]
        except (ValueError, IndexError) as e:
            rospy.logerr(f'Error processing state message: {str(e)}')

    def twist_callback(self, msg):
        """Store latest twist command"""
        self.last_twist = msg

    def clamp_position(self, pan, tilt):
        """Clamp pan and tilt values to min/max limits"""
        pan = max(min(pan, self.pan_max), self.pan_min)
        tilt = max(min(tilt, self.tilt_max), self.tilt_min)
        return pan, tilt

    def update_position(self, event):
        """Process latest twist command and update position"""
        if self.last_twist is None:
            return

        dt = 1.0 / self.update_rate

        # angular.z -> pan, linear.x -> tilt
        new_pan = self.current_pan + (self.last_twist.angular.z * dt)
        new_tilt = self.current_tilt + (-self.last_twist.linear.x * dt)

        # Clamp to limits
        new_pan, new_tilt = self.clamp_position(new_pan, new_tilt)

        # Command message
        cmd_msg = JointState()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.name = ['ptu_pan', 'ptu_tilt']
        cmd_msg.position = [new_pan, new_tilt]

        # Velocity hints (rad/s)
        cmd_msg.velocity = [
            abs(self.last_twist.angular.z / 2.0),
            abs(self.last_twist.linear.x / 2.0)
        ]

        self.cmd_pub.publish(cmd_msg)


if __name__ == '__main__':
    try:
        node = PTUTwistController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass