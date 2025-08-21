#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from rclpy.clock import Clock

class DummyController(Node):
    def __init__(self):
        super().__init__('dummy_controller')

        # This robot's properties from your URDF
        self.wheel_radius = 0.05 # meters

        # The robot's state
        self.x_pos = 0.0
        self.wheel_angle = 0.0
        self.last_time = self.get_clock().now()

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to publish updates
        self.timer = self.create_timer(0.02, self.update_robot_state) # 50 Hz

        self.get_logger().info('Dummy controller started. Publishing fake joint states and odom transform.')

    def update_robot_state(self):
        # Define a constant forward speed
        linear_speed = 0.2  # meters per second

        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update robot position and wheel angle
        distance_moved = linear_speed * dt
        self.x_pos += distance_moved
        self.wheel_angle += distance_moved / self.wheel_radius

        # 1. Publish Joint States
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'back_left_wheel_joint', 'back_right_wheel_joint'
        ]
        joint_state_msg.position = [self.wheel_angle, self.wheel_angle, self.wheel_angle, self.wheel_angle]
        self.joint_pub.publish(joint_state_msg)

        # 2. Publish TF Transform (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        # No rotation for moving in a straight line
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DummyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()