#!/usr/bin/env python3
"""Low-battery audible alarm.

Moved out of ugv_driver.py: now that the ros2_control hardware interface
(ugv_hardware) owns the UART and publishes /voltage, this small standalone node
keeps the audio/subprocess work off the real-time control thread.
"""
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

WAV_PATH = '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'


class BatteryAlarm(Node):
    def __init__(self):
        super().__init__('battery_alarm')
        self.declare_parameter('threshold', 9.0)
        self.declare_parameter('audio_device', 'plughw:3,0')
        self.declare_parameter('wav_path', WAV_PATH)
        self.threshold = self.get_parameter('threshold').value
        self.audio_device = self.get_parameter('audio_device').value
        self.wav_path = self.get_parameter('wav_path').value
        self.voltage_sub = self.create_subscription(
            Float32, 'voltage', self.voltage_callback, 10)

    def voltage_callback(self, msg):
        # Match the legacy ugv_driver behaviour: ignore the 0 V startup reading.
        if 0.1 < msg.data < self.threshold:
            if os.path.exists(self.wav_path):
                subprocess.run(['aplay', '-D', self.audio_device, self.wav_path])
            else:
                self.get_logger().warn(
                    f'Low battery ({msg.data:.2f} V) but wav not found: {self.wav_path}')
            time.sleep(5)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryAlarm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
