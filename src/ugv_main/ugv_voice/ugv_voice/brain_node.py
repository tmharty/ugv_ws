"""voice_brain: transcript -> validated intent -> speech + Behavior goal.

Flow per utterance:
  1. stop lexicon (redundant with the ear's scanner) -> behavior/estop, done
  2. pending go_to_point confirmation? consume a yes/no
  3. Brain backend parses text into a raw intent dict
  4. intent_schema.validate() — the ONLY path to motion
  5. speak the scripted reply, then (speak-then-act) send the Behavior goal
     after a short delay; the confirmation speech is the reaction window

The delay uses a stop-generation counter: an estop between speech and send
invalidates the pending goal, so a stop can never lose the race to a queued
motion.
"""

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import Trigger
from ugv_interface.action import Behavior
from ugv_interface.msg import Say, Transcript

from . import intent_schema, responses
from .brains import make_brain
from .dialog_context import DialogContext


class BrainNode(Node):
    def __init__(self):
        super().__init__('voice_brain')

        self.declare_parameter('brain_backend', 'rules')
        self.declare_parameter('speak_before_act_s', 1.2)
        self.declare_parameter('confirm_timeout_s', 15.0)
        self.declare_parameter('max_motions_per_minute', 12)

        backend = str(self.get_parameter('brain_backend').value)
        self.speak_before_act_s = float(self.get_parameter('speak_before_act_s').value)

        self.brain = make_brain(backend, self.get_logger())
        self.dialog = DialogContext(
            confirm_timeout_s=float(self.get_parameter('confirm_timeout_s').value))
        self.rate_limiter = intent_schema.CommandRateLimiter(
            max_commands=int(self.get_parameter('max_motions_per_minute').value))

        self.say_pub = self.create_publisher(Say, '/voice/say', 10)
        self.led_pub = self.create_publisher(Float32MultiArray, 'ugv/led_ctrl', 10)
        self.behavior_client = ActionClient(self, Behavior, 'behavior')
        self.estop_client = self.create_client(Trigger, 'behavior/estop')

        self.create_subscription(Transcript, '/voice/transcript',
                                 self.transcript_callback, 10)
        self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

        self.voltage = None
        self._stop_gen = 0
        self._lock = threading.Lock()

        self.get_logger().info(f'voice_brain up — backend: {self.brain.name}')

    # --- callbacks --------------------------------------------------------

    def voltage_callback(self, msg):
        self.voltage = float(msg.data)

    def transcript_callback(self, msg):
        text = msg.text.strip()

        # Stop path first, before any parsing. The ear already fired estop if
        # msg.stop_word is set; calling twice is harmless and covers brains
        # that map other phrasings ("that's enough") onto stop later.
        if msg.stop_word or (text and intent_schema.contains_stop_word(text)):
            self._do_estop(say_it=True)
            return

        if not text:
            self._say('heard_nothing')
            return

        raw = self._parse(text)

        # A pending go_to_point confirmation eats yes/no; any other command
        # implicitly cancels it and is processed normally.
        pending = self.dialog.pending()
        if pending is not None:
            intent_name = raw.get('intent') if isinstance(raw, dict) else None
            if intent_name == 'affirm':
                confirmed = self.dialog.resolve(True)
                self._say('nav_confirmed', **confirmed.params)
                self._send_behavior_later(confirmed)
                return
            if intent_name == 'deny':
                self.dialog.resolve(False)
                self._say('nav_cancelled')
                return
            self.dialog.cancel()

        self._act_on(intent_schema.validate(raw))

    def _parse(self, text):
        try:
            return self.brain.parse(text)
        except Exception as e:
            self.get_logger().error(f'brain {self.brain.name} raised: {e}')
            return {'intent': 'unknown'}

    # --- intent execution -------------------------------------------------

    def _act_on(self, intent):
        if intent.rejected_reason:
            self.get_logger().warn(f'rejected intent: {intent.rejected_reason}')
        for note in intent.clamp_notes:
            self.get_logger().warn(f'clamped: {note}')

        if intent.is_stop:
            self._do_estop(say_it=True)
            return

        if intent.name in ('led_on', 'led_off', 'led_blink'):
            self._do_led(intent.name)
            self._say(intent.reply_key)
            return

        if intent.name == 'battery_status':
            if self.voltage is None:
                self._say('battery_status_unknown')
            else:
                self._say('battery_status', voltage=self.voltage)
            return

        if intent.behavior_json is None:
            self._say(intent.reply_key, **intent.params)
            return

        # Motion / nav from here on.
        if not self.rate_limiter.allow():
            self.get_logger().warn('motion rate limit hit')
            self._say('rate_limited')
            return

        if not self.behavior_client.server_is_ready():
            self.get_logger().error('behavior action server not available')
            self._say('not_ready')
            return

        if intent.requires_confirmation:
            self.dialog.request_confirmation(intent)
            self._say(intent.reply_key, **intent.params)
            return

        self._say(intent.reply_key, **intent.params)
        self._send_behavior_later(intent)

    def _send_behavior_later(self, intent):
        """Speak-then-act: delay the goal so the ack is audible before wheels
        move, but abort silently if a stop lands during the delay."""
        with self._lock:
            gen = self._stop_gen

        def send():
            with self._lock:
                if gen != self._stop_gen:
                    self.get_logger().warn(
                        f'{intent.name}: dropped — stop requested during '
                        'speak-then-act delay')
                    return
            goal = Behavior.Goal()
            goal.command = intent.behavior_json
            self.get_logger().info(f'sending behavior goal: {intent.behavior_json}')
            future = self.behavior_client.send_goal_async(goal)
            future.add_done_callback(self._goal_response)

        threading.Timer(self.speak_before_act_s, send).start()

    def _goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('behavior goal rejected')

    def _do_estop(self, say_it):
        with self._lock:
            self._stop_gen += 1
        self.dialog.cancel()
        if say_it:
            self._say('estop', priority=Say.PRIORITY_SAFETY)
        if self.estop_client.service_is_ready():
            self.estop_client.call_async(Trigger.Request())
        else:
            self.get_logger().error('behavior/estop service not available!')

    def _do_led(self, name):
        if name == 'led_blink':
            # Three cycles, driven by timers so the executor is never blocked.
            for i in range(6):
                level = 255.0 if i % 2 == 0 else 0.0
                threading.Timer(0.4 * i, self._publish_led, args=(level,)).start()
        else:
            self._publish_led(255.0 if name == 'led_on' else 0.0)

    def _publish_led(self, level):
        msg = Float32MultiArray()
        msg.data = [level, level]  # IO4, IO5
        self.led_pub.publish(msg)

    def _say(self, key, priority=Say.PRIORITY_NORMAL, **slots):
        msg = Say()
        msg.key = key
        msg.text = responses.render(key, **slots)
        msg.priority = priority
        self.say_pub.publish(msg)
        self.get_logger().info(f'say[{key}]: {msg.text}')


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
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
