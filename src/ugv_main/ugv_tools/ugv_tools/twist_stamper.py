#!/usr/bin/env python3
"""Twist -> TwistStamped bridge.

On Jazzy, diff_drive_controller subscribes to ~/cmd_vel as geometry_msgs/TwistStamped
only (the Humble `use_stamped_vel` parameter was removed). The UGV's velocity
publishers (joy_ctrl, keyboard_ctrl, behavior_ctrl, Nav2) all still emit plain
geometry_msgs/Twist. This node converts them: it subscribes to `cmd_vel_in` (Twist)
and republishes on `cmd_vel_out` (TwistStamped), stamping each message with the
current time and a configurable frame_id.

In bringup_ros2_control.launch.py it is wired cmd_vel_in:=/cmd_vel,
cmd_vel_out:=/diff_cont/cmd_vel so every existing plain-Twist publisher keeps working.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('frame_id', 'base_footprint')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
        self.sub = self.create_subscription(Twist, 'cmd_vel_in', self.callback, 10)

    def callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.pub.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
