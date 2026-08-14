"""Behavior action server: the single motion seam for AI/voice commands.

Every AI- or voice-issued motion passes through this node, so it enforces the
safety envelope itself rather than trusting callers:

- All numeric goal values are clamped to ROS parameters (which are themselves
  clamped to hard ceilings, so a bad config file cannot re-enable unsafe speeds).
- Motion loops are rate-limited and abort on stop request, stale /odom, or a
  wall-clock timeout — the robot can never keep driving on frozen feedback.
- A `behavior/estop` Trigger service preempts the current motion, drains queued
  commands, cancels any in-flight Nav2 goal, and publishes zero twists.

Preemption uses a "stop generation" counter instead of a clearable event: each
command captures the counter at enqueue time and aborts if it has changed. A
stop therefore kills everything enqueued before it, while commands issued after
it run normally — with no clear-the-flag race in between.
"""

import json
import math
import queue
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from ugv_interface.action import Behavior


class BehaviorController(Node):
    # name: (default, min, hard ceiling). Parameter values outside these bounds
    # are clamped so misconfiguration cannot exceed the safety envelope.
    PARAM_BOUNDS = {
        'max_linear_speed': (0.15, 0.01, 0.5),    # m/s — kid-slow default
        'max_angular_speed': (0.5, 0.05, 1.5),    # rad/s
        'max_distance': (1.0, 0.05, 5.0),         # m per command
        'max_angle_deg': (360.0, 5.0, 360.0),     # deg per command
        'loop_rate_hz': (20.0, 5.0, 100.0),
        'odom_timeout': (0.5, 0.1, 5.0),          # s without /odom before abort
    }

    def __init__(self):
        super().__init__('behavior_ctrl')

        for name, (default, _, _) in self.PARAM_BOUNDS.items():
            self.declare_parameter(name, default)
        self.declare_parameter('use_nav2_action', True)

        self.max_linear_speed = self._bounded_param('max_linear_speed')
        self.max_angular_speed = self._bounded_param('max_angular_speed')
        self.max_distance = self._bounded_param('max_distance')
        self.max_angle_deg = self._bounded_param('max_angle_deg')
        self.loop_rate_hz = self._bounded_param('loop_rate_hz')
        self.odom_timeout = self._bounded_param('odom_timeout')
        self.use_nav2_action = bool(self.get_parameter('use_nav2_action').value)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self.robot_pose_callback, 10)
        self.behavior_action_server = ActionServer(self, Behavior, 'behavior', self.execute_callback)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Legacy fire-and-forget fallback for pub_nav_point (use_nav2_action:=false)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.estop_service = self.create_service(Trigger, 'behavior/estop', self.estop_callback)

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_handle = None
        self.nav_goal_lock = threading.Lock()

        # Odometry state. position/yaw are replaced (not mutated) per message,
        # so a reference taken by the worker thread is a stable snapshot.
        self.position = None
        self.yaw = 0.0
        self._last_odom_mono = None
        self.map_pose = None
        self.points = {}

        # Bumped by any stop request; only ever incremented from the (single
        # threaded) executor, read by the worker thread.
        self._stop_gen = 0

        self.behaviors = {
            "drive_on_heading": self.drive_on_heading,
            "back_up": self.back_up,
            "spin": self.spin,
            "stop": self.stop,
            "save_map_point": self.save_map_point,
            "pub_nav_point": self.pub_nav_point,
        }
        # Behaviors whose data value is a saved map-point name rather than a number
        self.point_behaviors = {"save_map_point", "pub_nav_point"}
        # Short aliases for saved map points (preserves the original a..g payloads).
        # A value not found here is passed through unchanged, so full names work too.
        self.point_names = {
            "a": "point_a",
            "b": "point_b",
            "c": "point_c",
            "d": "point_d",
            "e": "point_e",
            "f": "point_f",
            "g": "point_g",
        }

        self.command_queue = queue.Queue()
        self.executor_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.executor_thread.start()

    # --- parameters / validation -----------------------------------------

    def _bounded_param(self, name):
        default, lo, hi = self.PARAM_BOUNDS[name]
        raw = self.get_parameter(name).value
        try:
            value = float(raw)
        except (TypeError, ValueError):
            value = float('nan')
        if not math.isfinite(value):
            self.get_logger().warn(f'Parameter {name}={raw!r} invalid — using default {default}')
            return default
        clamped = min(max(value, lo), hi)
        if clamped != value:
            self.get_logger().warn(f'Parameter {name}={value} outside [{lo}, {hi}] — clamped to {clamped}')
        return clamped

    def _as_finite_float(self, label, value):
        try:
            v = float(value)
        except (TypeError, ValueError):
            self.get_logger().error(f'{label}: non-numeric value {value!r} — dropping command')
            return None
        if not math.isfinite(v):
            self.get_logger().error(f'{label}: non-finite value {v} — dropping command')
            return None
        return v

    def _validated_distance(self, label, value):
        v = self._as_finite_float(label, value)
        if v is None:
            return None
        if v < 0:
            self.get_logger().warn(f'{label}: negative distance {v} — using {abs(v)}')
            v = abs(v)
        if v == 0.0:
            self.get_logger().warn(f'{label}: zero distance — dropping command')
            return None
        if v > self.max_distance:
            self.get_logger().warn(f'{label}: distance {v:.2f} m clamped to max_distance {self.max_distance:.2f} m')
            v = self.max_distance
        return v

    def _validated_angle(self, value):
        v = self._as_finite_float('spin', value)
        if v is None:
            return None
        if v == 0.0:
            self.get_logger().warn('spin: zero angle — dropping command')
            return None
        if abs(v) > self.max_angle_deg:
            clamped = math.copysign(self.max_angle_deg, v)
            self.get_logger().warn(f'spin: angle {v:.1f}° clamped to {clamped:.1f}°')
            v = clamped
        return v

    # --- callbacks --------------------------------------------------------

    def robot_pose_callback(self, msg):
        self.map_pose = msg.pose

    def odom_callback(self, msg):
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        q0 = msg.pose.pose.orientation.w
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)

        self.position = msg.pose.pose.position
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        # Monotonic wall clock, deliberately not node time: staleness must
        # still trip if the sim (and therefore /clock) dies mid-motion.
        self._last_odom_mono = time.monotonic()

    def _odom_age(self):
        if self._last_odom_mono is None:
            return float('inf')
        return time.monotonic() - self._last_odom_mono

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received behavior goal: {goal_handle.request.command}')
        gen = self._stop_gen

        try:
            json_list = json.loads(goal_handle.request.command)
            if not isinstance(json_list, list):
                raise ValueError('command must be a JSON list')
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f'Rejecting malformed behavior goal: {e}')
            goal_handle.abort()
            result = Behavior.Result()
            result.result = False
            return result

        accepted = 0
        for json_data in json_list:
            if not isinstance(json_data, dict) or 'type' not in json_data or 'data' not in json_data:
                self.get_logger().warn(f'Ignoring malformed command entry: {json_data!r}')
                continue
            command_type = json_data['type']
            data_value = json_data['data']

            if command_type not in self.behaviors:
                self.get_logger().warn(f'Ignoring unknown behavior: {command_type}')
                continue

            # A stop preempts the current motion and drops everything queued
            # (including the rest of this goal) — it never waits in the queue.
            if command_type == "stop":
                self._request_stop('behavior goal')
                accepted += 1
                break

            # Clamp/validate here, at the trust boundary, so only vetted
            # numeric values ever reach the queue.
            if command_type in ("drive_on_heading", "back_up"):
                data_value = self._validated_distance(command_type, data_value)
            elif command_type == "spin":
                data_value = self._validated_angle(data_value)
            elif command_type in self.point_behaviors:
                data_value = str(data_value)
            if data_value is None:
                continue

            self.command_queue.put((command_type, data_value, gen))
            accepted += 1

        result = Behavior.Result()
        if accepted == 0:
            self.get_logger().error('No valid commands in behavior goal')
            goal_handle.abort()
            result.result = False
        else:
            goal_handle.succeed()
            result.result = True
        return result

    # --- stop / estop -----------------------------------------------------

    def _request_stop(self, source):
        """Preempt current motion, drop queued commands, cancel Nav2 goal."""
        self._stop_gen += 1
        dropped = 0
        while True:
            try:
                self.command_queue.get_nowait()
                self.command_queue.task_done()
                dropped += 1
            except queue.Empty:
                break
        self._cancel_nav_goal()
        self._publish_stop()
        self.get_logger().warn(f'STOP requested via {source}; dropped {dropped} queued command(s)')
        return dropped

    def estop_callback(self, request, response):
        dropped = self._request_stop('behavior/estop service')
        # Keep asserting zero for ~0.5 s so the stop wins any publish race.
        period = 1.0 / self.loop_rate_hz
        for _ in range(max(1, int(0.5 * self.loop_rate_hz))):
            self._publish_stop()
            time.sleep(period)
        response.success = True
        response.message = f'stopped; dropped {dropped} queued command(s)'
        return response

    def _publish_stop(self):
        self.velocity_publisher.publish(Twist())

    def _cancel_nav_goal(self):
        with self.nav_goal_lock:
            handle = self.nav_goal_handle
        if handle is not None:
            self.get_logger().warn('Cancelling in-flight Nav2 goal')
            handle.cancel_goal_async()

    # --- command execution ------------------------------------------------

    def process_commands(self):
        while True:
            command = self.command_queue.get()
            if command is None:
                break
            self.execute_behavior(command)
            self.command_queue.task_done()

    def execute_behavior(self, command):
        command_type, data_value, gen = command
        if gen != self._stop_gen:
            self.get_logger().warn(f'Skipping {command_type}: enqueued before a stop request')
            return

        func = self.behaviors.get(command_type)
        if func is None:
            # Should not happen (validated on enqueue), but guard anyway
            self.get_logger().error(f'Unknown behavior: {command_type}')
            return

        try:
            if command_type == "stop":
                func()
            elif command_type == "save_map_point":
                func(self.point_names.get(data_value, data_value))
            elif command_type == "pub_nav_point":
                func(self.point_names.get(data_value, data_value), gen)
            else:
                func(data_value, gen)
        except Exception as e:
            self.get_logger().error(f'Error executing behavior: {e}')
            self.get_logger().error(f'Command: {command_type}({data_value})')
            self._publish_stop()

    def _run_motion(self, label, twist, timeout, done, gen):
        """Publish twist at loop_rate_hz until done() returns True.

        Aborts on stop request, stale /odom, or timeout. Always finishes by
        publishing a zero twist. Returns True iff the motion completed.
        """
        period = 1.0 / self.loop_rate_hz
        start = time.monotonic()
        try:
            while True:
                if gen != self._stop_gen:
                    self.get_logger().warn(f'{label}: aborted by stop request')
                    return False
                age = self._odom_age()
                if age > self.odom_timeout:
                    self.get_logger().error(f'{label}: /odom stale ({age:.2f} s) — aborting motion')
                    return False
                if done():
                    self.get_logger().info(f'{label}: completed')
                    return True
                if time.monotonic() - start > timeout:
                    self.get_logger().error(f'{label}: timed out after {timeout:.1f} s — aborting motion')
                    return False
                self.velocity_publisher.publish(twist)
                time.sleep(period)
        finally:
            self._publish_stop()

    def drive_on_heading(self, distance, gen):
        self._drive_linear('drive_on_heading', distance, 1.0, gen)

    def back_up(self, distance, gen):
        self._drive_linear('back_up', distance, -1.0, gen)

    def _drive_linear(self, label, distance, direction, gen):
        if self._odom_age() > self.odom_timeout:
            self.get_logger().error(f'{label}: no fresh /odom — refusing to move')
            return
        start = self.position
        twist = Twist()
        twist.linear.x = direction * self.max_linear_speed

        def done():
            moved = math.hypot(self.position.x - start.x, self.position.y - start.y)
            self.get_logger().info(f'{label}: moved {moved:.2f}/{distance:.2f} m',
                                   throttle_duration_sec=1.0)
            return moved >= distance

        timeout = distance / self.max_linear_speed * 2.0 + 2.0
        self._run_motion(label, twist, timeout, done, gen)

    def spin(self, angle_deg, gen):
        if self._odom_age() > self.odom_timeout:
            self.get_logger().error('spin: no fresh /odom — refusing to move')
            return
        target = abs(math.radians(angle_deg))
        twist = Twist()
        twist.angular.z = math.copysign(self.max_angular_speed, angle_deg)

        # Accumulate wrapped yaw increments so rotations >180° are measurable.
        state = {'prev': self.yaw, 'total': 0.0}

        def done():
            yaw = self.yaw
            delta = (yaw - state['prev'] + math.pi) % (2 * math.pi) - math.pi
            state['total'] += delta
            state['prev'] = yaw
            self.get_logger().info(
                f'spin: rotated {math.degrees(abs(state["total"])):.0f}/{math.degrees(target):.0f}°',
                throttle_duration_sec=1.0)
            return abs(state['total']) >= target

        timeout = target / self.max_angular_speed * 2.0 + 2.0
        self._run_motion('spin', twist, timeout, done, gen)

    def stop(self):
        self._publish_stop()

    # --- map points / navigation ------------------------------------------

    def save_map_point(self, point):
        if self.map_pose is not None:
            self.points[point] = self.map_pose
            self.get_logger().info(f'Added point "{point}": {self.map_pose}')
            self.save_points_to_file()
        else:
            self.get_logger().warn('No current pose available to create map point.')

    def save_points_to_file(self):
        with open('/home/ws/ugv_ws/map_points.txt', 'w') as file:
            for point_name, pose in self.points.items():
                file.write(f'{point_name}: Position(x={pose.position.x}, y={pose.position.y}, z={pose.position.z}), Orientation(x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w})\n')
        self.get_logger().info('Saved points to map_points.txt')

    def pub_nav_point(self, point, gen):
        if point not in self.points:
            self.get_logger().warn(f'Point "{point}" not found in saved points.')
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self.points[point]

        if not self.use_nav2_action:
            # Legacy fallback: fire-and-forget publish (estop cannot cancel it)
            self.goal_publisher.publish(goal_pose)
            self.get_logger().info(f'Sent goal to /goal_pose: {goal_pose.pose.position}')
            return

        if not self.nav_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('navigate_to_pose action server not available — goal not sent')
            return
        if gen != self._stop_gen:
            self.get_logger().warn('pub_nav_point: aborted by stop request before send')
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        self.get_logger().info(f'Sending NavigateToPose goal: {goal_pose.pose.position}')
        send_future = self.nav_action_client.send_goal_async(nav_goal)
        send_future.add_done_callback(lambda f: self._nav_goal_response_callback(f, gen))

    def _nav_goal_response_callback(self, future, gen):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('NavigateToPose goal rejected')
            return
        with self.nav_goal_lock:
            self.nav_goal_handle = handle
        # A stop may have arrived while the goal was in flight — cancel now.
        if gen != self._stop_gen:
            self.get_logger().warn('Stop requested while Nav2 goal was in flight — cancelling')
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(
            lambda f: self._nav_result_callback(f, handle))

    def _nav_result_callback(self, future, handle):
        with self.nav_goal_lock:
            if self.nav_goal_handle is handle:
                self.nav_goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation goal cancelled')
        else:
            self.get_logger().warn(f'Navigation goal finished with status {status}')

    # --- lifecycle ---------------------------------------------------------

    def destroy_node(self):
        self._stop_gen += 1
        self.command_queue.put(None)
        self.executor_thread.join(timeout=2.0)
        self._publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
