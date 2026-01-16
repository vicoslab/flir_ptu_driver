#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
from ptu_driver import PTU
from threading import Lock
from tf.transformations import quaternion_from_euler

def deg2rad(deg):
    return deg * math.pi / 180.0

def rad2deg(rad):
    return rad * 180.0 / math.pi


class PTUNode:
    def __init__(self):
        rospy.init_node('ptu_node')

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Frame names
        self.base_frame = 'ptu_base'
        self.yaw_frame = 'ptu_yaw'
        self.pitch_frame = 'ptu_pitch'

        # Parameters
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baud = rospy.get_param('~baud', 9600)
        self.rate = rospy.get_param('~publishing_rate', 20.0)

        # Initialize PTU
        try:
            self.ptu = PTU(port=port, baud=baud)
            rospy.loginfo(f'Connected to PTU on {port}')
        except Exception as e:
            rospy.logerr(f'Failed to connect to PTU: {str(e)}')
            raise

        # Store conversion factors for speed calculations
        # Convert from rad/s to positions/sec
        self.pan_speed_conv = abs(self.ptu.panAngleToPosition(rad2deg(1.0)))
        self.tilt_speed_conv = abs(self.ptu.tiltAngleToPosition(rad2deg(1.0)))

        # Publishers and subscribers
        self.state_pub = rospy.Publisher(
            'ptu/state',
            JointState,
            queue_size=10
        )

        self.cmd_sub = rospy.Subscriber(
            'ptu/cmd',
            JointState,
            self.cmd_callback,
            queue_size=10
        )

        # Mutex for thread safety
        self.ptu_lock = Lock()

        # Timer for publishing state
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate),
            self.publish_state
        )

        rospy.loginfo('PTU node initialized')

    def publish_state(self, event):
        """Publish current PTU state as JointState and TF"""
        try:
            with self.ptu_lock:
                pan_pos, tilt_pos = self.ptu.getPosition()

            # Convert positions to angles (radians)
            pan_angle_deg = pan_pos * (self.ptu.panResolution / 3600.0)
            tilt_angle_deg = tilt_pos * (self.ptu.tiltResolution / 3600.0)

            pan_angle_rad = deg2rad(pan_angle_deg)   # yaw
            tilt_angle_rad = deg2rad(tilt_angle_deg) # pitch

            now = rospy.Time.now()

            # ---- JointState ----
            msg = JointState()
            msg.header.stamp = now
            msg.name = ['ptu_pan', 'ptu_tilt']
            msg.position = [pan_angle_rad, tilt_angle_rad]
            self.state_pub.publish(msg)

            # ---- TF: base -> yaw ----
            t_base_yaw = TransformStamped()
            t_base_yaw.header.stamp = now
            t_base_yaw.header.frame_id = self.base_frame
            t_base_yaw.child_frame_id = self.yaw_frame

            t_base_yaw.transform.translation.x = 0.04
            t_base_yaw.transform.translation.y = -0.015
            t_base_yaw.transform.translation.z = 0.05

            q_yaw = quaternion_from_euler(0.0, 0.0, pan_angle_rad)
            t_base_yaw.transform.rotation.x = q_yaw[0]
            t_base_yaw.transform.rotation.y = q_yaw[1]
            t_base_yaw.transform.rotation.z = q_yaw[2]
            t_base_yaw.transform.rotation.w = q_yaw[3]

            # ---- TF: yaw -> pitch ----
            t_yaw_pitch = TransformStamped()
            t_yaw_pitch.header.stamp = now
            t_yaw_pitch.header.frame_id = self.yaw_frame
            t_yaw_pitch.child_frame_id = self.pitch_frame

            t_yaw_pitch.transform.translation.x = 0.0
            t_yaw_pitch.transform.translation.y = 0.0
            t_yaw_pitch.transform.translation.z = 0.045

            q_pitch = quaternion_from_euler(0.0, -tilt_angle_rad, 0.0)
            t_yaw_pitch.transform.rotation.x = q_pitch[0]
            t_yaw_pitch.transform.rotation.y = q_pitch[1]
            t_yaw_pitch.transform.rotation.z = q_pitch[2]
            t_yaw_pitch.transform.rotation.w = q_pitch[3]

            self.tf_broadcaster.sendTransform([t_base_yaw, t_yaw_pitch])

        except Exception as e:
            rospy.logerr(f'Error publishing state/TF: {str(e)}')


    def cmd_callback(self, msg):
        """Handle incoming command messages"""
        try:
            if len(msg.position) != 2 or len(msg.name) != 2:
                rospy.logwarn('Invalid command message format')
                return

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

            if pan_angle_rad is None or tilt_angle_rad is None:
                rospy.logwarn('Missing pan or tilt values in command')
                return

            # Convert angles to positions
            pan_angle_deg = rad2deg(pan_angle_rad)
            tilt_angle_deg = rad2deg(tilt_angle_rad)

            pan_pos = self.ptu.panAngleToPosition(pan_angle_deg)
            tilt_pos = self.ptu.tiltAngleToPosition(tilt_angle_deg)

            with self.ptu_lock:
                if pan_velocity_rad is not None and tilt_velocity_rad is not None:
                    pan_speed = abs(int(pan_velocity_rad * self.pan_speed_conv))
                    tilt_speed = abs(int(tilt_velocity_rad * self.tilt_speed_conv))
                    self.ptu.setPositionAndSpeed(
                        pan_pos,
                        tilt_pos,
                        pan_speed,
                        tilt_speed
                    )
                else:
                    self.ptu.setPosition(pan_pos, tilt_pos)

        except Exception as e:
            rospy.logerr(f'Error processing command: {str(e)}')

    def cleanup(self):
        """Clean up PTU connection"""
        if hasattr(self, 'ptu'):
            try:
                self.ptu.halt()
            except Exception:
                pass
            del self.ptu


if __name__ == '__main__':
    node = None
    try:
        node = PTUNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.cleanup()
