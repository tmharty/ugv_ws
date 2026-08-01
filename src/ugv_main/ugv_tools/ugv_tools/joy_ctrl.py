#!/usr/bin/env python
# encoding: utf-8

import os
import time
import getpass
import threading
from time import sleep

import glob

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int32, Bool, Float32MultiArray

# pygame is only used to read the controller's *name*; the actual joystick input
# arrives on the /joy topic from joy_node. In a headless container pygame may be
# missing or unable to enumerate joysticks (SDL needs udev), so treat it as
# optional and fall back to sysfs below.
try:
    import pygame
except ImportError:
    pygame = None

def get_joystick_names():
    joystick_names = []

    # Preferred path: SDL/pygame (works on a desktop with a udev session).
    if pygame is not None:
        pygame.init()
        pygame.joystick.init()
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            joystick_names.append(joystick.get_name())
        pygame.quit()

    # Fallback: headless containers often can't enumerate joysticks via SDL/udev,
    # so read the kernel-reported name straight from sysfs (e.g. /dev/input/js0).
    if not joystick_names:
        for path in sorted(glob.glob('/sys/class/input/js*/device/name')):
            try:
                with open(path) as fh:
                    joystick_names.append(fh.read().strip())
            except OSError:
                pass

    return joystick_names
    
class JoyTeleop(Node):
	def __init__(self,name):
		super().__init__(name)
		self.Joy_active = True
		self.user_name = getpass.getuser()
		self.linear_Gear = 1
		self.angular_Gear = 1
		
		#create pub
		self.pub_cmdVel = self.create_publisher(Twist,'cmd_vel',  10)
		self.pub_JoyState = self.create_publisher(Bool,"JoyState",  10)
		self.pub_gimbal = self.create_publisher(JointState, 'ugv/joint_states', 10)
		self.pub_led = self.create_publisher(Float32MultiArray, 'ugv/led_ctrl', 10)

		#create sub
		self.sub_Joy = self.create_subscription(Joy,'joy', self.buttonCallback,10)
		
		#declare parameter and get the value
		self.declare_parameter('xspeed_limit',0.5)
		self.declare_parameter('yspeed_limit',0.5)
		self.declare_parameter('angular_speed_limit',1.0)
		self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
		self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
		self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value
		joysticks = get_joystick_names()
		self.joysticks = joysticks[0] if len(joysticks) != 0 else "no"
		# Per-controller input map. Named keys keep the growing set of indices readable:
		#   lin_gear/ang_gear = buttons that cycle the speed gears
		#   ang_axis          = stick axis used to turn the robot
		#   pan_axis/tilt_axis = D-pad axes that aim the camera gimbal
		#   recenter          = button that re-centers the gimbal (A)
		#   led_robot/led_cam = buttons that toggle the two LED channels (Y / B)
		self.switch_dict = {
			"Xbox 360 Controller":     {"lin_gear":9,"ang_gear":10,"ang_axis":3,
										"pan_axis":6,"tilt_axis":7,
										"recenter":0,"led_robot":3,"led_cam":1},
			"Microsoft X-Box 360 pad": {"lin_gear":9,"ang_gear":10,"ang_axis":3,
										"pan_axis":6,"tilt_axis":7,
										"recenter":0,"led_robot":3,"led_cam":1},
			# SHANWAN indices below are best-guess for pan/tilt/led; verify with
			# `ros2 topic echo /joy` on the actual pad and adjust if they differ.
			"SHANWAN Android Gamepad": {"lin_gear":13,"ang_gear":14,"ang_axis":2,
										"pan_axis":4,"tilt_axis":5,
										"recenter":0,"led_robot":3,"led_cam":1},
		}
		# Fall back to the Xbox layout for an unrecognized controller so a missing
		# entry degrades gracefully instead of raising KeyError.
		self.cfg = self.switch_dict.get(self.joysticks, self.switch_dict["Xbox 360 Controller"])

		# Gimbal state (radians), start centered. URDF limits: pan +/-3.14, tilt -0.523..1.571
		self.pan_rad = 0.0
		self.tilt_rad = 0.0
		self.pan_input = 0.0     # latest D-pad X (-1/0/1)
		self.tilt_input = 0.0    # latest D-pad Y
		self.gimbal_dirty = True # publish once at startup to center the servo
		self.GIMBAL_STEP = 0.03  # rad per tick (~0.6 rad/s at 20 Hz) - tunable
		self.PAN_MIN, self.PAN_MAX = -3.14, 3.14
		self.TILT_MIN, self.TILT_MAX = -0.523, 1.571

		# LED state and rising-edge tracking for the toggle/recenter buttons
		self.led_robot_on = False
		self.led_cam_on = False
		self.prev_buttons = []

		# Integrate the D-pad into smooth gimbal motion on a timer, because joy_linux
		# does not auto-repeat a held D-pad (it only publishes on input change).
		self.create_timer(0.05, self.gimbal_timer)   # 20 Hz

	def buttonCallback(self,joy_data):
		if not isinstance(joy_data, Joy): return
		self.handle_aux(joy_data)
		if self.user_name == "root": self.user_jetson(joy_data)
		else: self.user_pc(joy_data)
    
	def user_jetson(self, joy_data):
			#linear Gear control
		if joy_data.buttons[self.cfg["lin_gear"]] == 1:
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
			# angular Gear control
		if joy_data.buttons[self.cfg["ang_gear"]] == 1:
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
		xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
			#ylinear_speed = self.filter_data(joy_data.axes[2]) * self.yspeed_limit * self.linear_Gear
		ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
		angular_speed = self.filter_data(joy_data.axes[self.cfg["ang_axis"]]) * self.angular_speed_limit * self.angular_Gear
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		if self.Joy_active == True:
			self.pub_cmdVel.publish(twist)
        
	def user_pc(self, joy_data):
			# Gear control
		if joy_data.buttons[self.cfg["lin_gear"]] == 1:
			if self.linear_Gear == 1.0: self.linear_Gear = 1.0 / 3
			elif self.linear_Gear == 1.0 / 3: self.linear_Gear = 2.0 / 3
			elif self.linear_Gear == 2.0 / 3: self.linear_Gear = 1
		if joy_data.buttons[self.cfg["ang_gear"]] == 1:
			if self.angular_Gear == 1.0: self.angular_Gear = 1.0 / 4
			elif self.angular_Gear == 1.0 / 4: self.angular_Gear = 1.0 / 2
			elif self.angular_Gear == 1.0 / 2: self.angular_Gear = 3.0 / 4
			elif self.angular_Gear == 3.0 / 4: self.angular_Gear = 1.0
		xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
		ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
		angular_speed = self.filter_data(joy_data.axes[self.cfg["ang_axis"]]) * self.angular_speed_limit * self.angular_Gear
		if xlinear_speed > self.xspeed_limit: xlinear_speed = self.xspeed_limit
		elif xlinear_speed < -self.xspeed_limit: xlinear_speed = -self.xspeed_limit
		if ylinear_speed > self.yspeed_limit: ylinear_speed = self.yspeed_limit
		elif ylinear_speed < -self.yspeed_limit: ylinear_speed = -self.yspeed_limit
		if angular_speed > self.angular_speed_limit: angular_speed = self.angular_speed_limit
		elif angular_speed < -self.angular_speed_limit: angular_speed = -self.angular_speed_limit
		twist = Twist()
		twist.linear.x = xlinear_speed
		twist.linear.y = ylinear_speed
		twist.angular.z = angular_speed
		self.pub_cmdVel.publish(twist)
        
	def filter_data(self, value):
		if abs(value) < 0.2: value = 0
		return value

	def rising_edge(self, buttons, idx):
		# True only on the press transition. joy_linux re-sends the full button
		# state on any input change, so a plain "== 1" check would fire repeatedly
		# while the button (or any other input) is held.
		now = idx < len(buttons) and buttons[idx] == 1
		was = idx < len(self.prev_buttons) and self.prev_buttons[idx] == 1
		return now and not was

	def handle_aux(self, joy):
		# Gimbal + LED handling shared by both the jetson and pc drive paths.
		c = self.cfg
		# D-pad -> gimbal input (-1/0/1). Flip a sign here if a direction feels reversed.
		# turns the camera right, so the raw value would mirror the stick.
		self.pan_input  = -joy.axes[c["pan_axis"]] if c["pan_axis"]  < len(joy.axes) else 0.0
		self.tilt_input = joy.axes[c["tilt_axis"]] if c["tilt_axis"] < len(joy.axes) else 0.0
		# Re-center the gimbal
		if self.rising_edge(joy.buttons, c["recenter"]):
			self.pan_rad = 0.0
			self.tilt_rad = 0.0
			self.gimbal_dirty = True
		# LED toggles (two independent channels)
		if self.rising_edge(joy.buttons, c["led_robot"]):
			self.led_robot_on = not self.led_robot_on
			self.publish_leds()
		if self.rising_edge(joy.buttons, c["led_cam"]):
			self.led_cam_on = not self.led_cam_on
			self.publish_leds()
		self.prev_buttons = list(joy.buttons)

	def publish_leds(self):
		m = Float32MultiArray()
		# data[0]=IO4 (robot lights), data[1]=IO5 (camera light).
		# NOTE: verify which physical LEDs are on which channel and swap if reversed.
		m.data = [255.0 if self.led_robot_on else 0.0,
				  255.0 if self.led_cam_on  else 0.0]
		self.pub_led.publish(m)

	def gimbal_timer(self):
		# Integrate the held D-pad into pan/tilt at a fixed rate, clamped to the
		# URDF joint limits. Publish only when moving/changed to avoid spamming
		# the serial bus when the gimbal is idle.
		if abs(self.pan_input) > 0.5 or abs(self.tilt_input) > 0.5:
			self.pan_rad  = min(max(self.pan_rad  + self.pan_input  * self.GIMBAL_STEP,
									self.PAN_MIN),  self.PAN_MAX)
			self.tilt_rad = min(max(self.tilt_rad + self.tilt_input * self.GIMBAL_STEP,
									self.TILT_MIN), self.TILT_MAX)
			self.gimbal_dirty = True
		if self.gimbal_dirty:
			js = JointState()
			js.header.stamp = self.get_clock().now().to_msg()
			js.name = ['pt_base_link_to_pt_link1', 'pt_link1_to_pt_link2']
			js.position = [float(self.pan_rad), float(self.tilt_rad)]
			self.pub_gimbal.publish(js)
			self.gimbal_dirty = False

def main():
	rclpy.init()
	joy_ctrl = JoyTeleop('joy_ctrl')
	rclpy.spin(joy_ctrl)	
	
main()		