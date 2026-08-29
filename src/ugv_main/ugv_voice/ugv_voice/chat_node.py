"""voice_chat: /voice/transcript -> Ollama LLM (with tools) -> speech + action.

The hub of the voice stack. The model converses freely and may *propose*
tool calls (motion, LEDs, battery, saved points). Every proposal passes
tools.py -> intent_schema.validate (allow-list + clamps) before anything
is dispatched, and motion still goes through behavior_ctrl's own clamps,
odom-staleness abort and estop. Speech safety is the persona prompt;
action safety is deterministic and lives outside the model.

Turn flow:
  1. stop word (ear flag or lexicon)  -> estop + barge-in, done, outside the LLM
  2. pending go_to_point confirmation -> consume a yes/no
  3. ToolLoop: stream the reply, speak prose sentence-by-sentence, validate
     and dispatch each tool call, feed the result back so the model can
     narrate; capped at max_tool_rounds per user turn
  4. LLM unreachable -> RuleBrain parses the transcript through the same
     validator and dispatch; "my chat brain is not answering" is spoken.
     Basic motion and stop keep working with the daemon dead.

Motion dispatch is speak-then-act: the acknowledgement (the model's own
sentence, else a per-tool template) is spoken first and the Behavior goal
is sent after speak_before_act_s. A stop-generation counter, bumped by any
estop, drops a goal still waiting in that window.

record_replay (Phase 3) is orchestration only: spoken countdown, wait for
the mouth to fall silent, call the ear's voice/record service, queue the
wav playbacks on /voice/say with speed factors. Refused while anything is
moving or a goal is pending (recording blinds the stop-word scanner).

Session flow is unchanged from talk-only chat: publish /voice/listen_once
and speak; the ear is re-armed when the mouth falls silent (auto_listen);
the session ends on a goodbye phrase or max_silent_turns empty listens.
"""

import glob
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray
from std_srvs.srv import Trigger
from ugv_interface.action import Behavior
from ugv_interface.msg import Say, Transcript
from ugv_interface.srv import Record

from . import intent_schema, responses, tools
from .brains.rule_brain import RuleBrain
from .chat_session import (ChatHistory, DEFAULT_SYSTEM_PROMPT, ToolLoop,
                           TurnFailed, is_end_phrase)
from .dialog_context import DialogContext


class ChatNode(Node):
    def __init__(self):
        super().__init__('voice_chat')

        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('ollama_model', 'granite4:3b')
        # First request includes model load on the Jetson — allow for it.
        self.declare_parameter('request_timeout_s', 120.0)
        self.declare_parameter('connect_timeout_s', 5.0)
        self.declare_parameter('keep_alive', '10m')
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_reply_tokens', 120)
        self.declare_parameter('max_history_turns', 12)
        self.declare_parameter('system_prompt', '')  # '' = built-in default
        self.declare_parameter('auto_listen', True)
        self.declare_parameter('relisten_delay_s', 0.7)
        self.declare_parameter('max_silent_turns', 2)
        # Tools / action safety (mirrors voice_brain's params).
        self.declare_parameter('tools_enabled', True)
        self.declare_parameter('max_tool_rounds', 3)
        self.declare_parameter('speak_before_act_s', 1.2)
        self.declare_parameter('confirm_timeout_s', 15.0)
        self.declare_parameter('max_motions_per_minute', 12)
        self.declare_parameter('offline_fallback', True)
        self.declare_parameter('recordings_dir', '/tmp/ugv_voice_recordings')

        system_prompt = (str(self.get_parameter('system_prompt').value)
                         or DEFAULT_SYSTEM_PROMPT)
        self._history = ChatHistory(
            system_prompt,
            max_turns=int(self.get_parameter('max_history_turns').value))
        self._loop = ToolLoop(
            self._history,
            max_tool_rounds=int(self.get_parameter('max_tool_rounds').value),
            tools_enabled=bool(self.get_parameter('tools_enabled').value))
        self.speak_before_act_s = float(self.get_parameter('speak_before_act_s').value)
        self.rule_brain = RuleBrain()
        self.dialog = DialogContext(
            confirm_timeout_s=float(self.get_parameter('confirm_timeout_s').value))
        self.rate_limiter = intent_schema.CommandRateLimiter(
            max_commands=int(self.get_parameter('max_motions_per_minute').value))

        self.say_pub = self.create_publisher(Say, '/voice/say', 10)
        self.listen_pub = self.create_publisher(Empty, '/voice/listen_once', 10)
        self.led_pub = self.create_publisher(Float32MultiArray, 'ugv/led_ctrl', 10)
        self.behavior_client = ActionClient(self, Behavior, 'behavior')
        self.estop_client = self.create_client(Trigger, 'behavior/estop')
        self.record_client = self.create_client(Record, 'voice/record')
        self.create_subscription(
            Bool, 'behavior/motion_active', self.motion_active_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Transcript, '/voice/transcript',
                                 self.transcript_callback, 10)
        self.create_subscription(Bool, '/voice/speaking',
                                 self.speaking_callback, 10)
        self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

        self._queue = queue.Queue()
        self._generation = 0        # bumped on barge-in; worker drops stale work
        self._stop_gen = 0          # bumped on any estop; drops pending goals
        self._voltage = None
        self._motion_active = False
        self._pending_goals = 0     # speak-then-act goals not yet sent
        self._last_goal_sent = 0.0  # monotonic; grace until motion_active catches up
        self.MOTION_FLAG_GRACE_S = 1.5
        self._speaking = False
        self._session_active = False
        self._silent_turns = 0
        self._awaiting_relisten = False
        self._relisten_timer = None
        self._lock = threading.Lock()

        self.recordings_dir = str(self.get_parameter('recordings_dir').value)
        self._sweep_recordings()

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'voice_chat up — model %r at %s, tools %s.'
            ' Publish /voice/listen_once to start a conversation.'
            % (str(self.get_parameter('ollama_model').value),
               str(self.get_parameter('ollama_url').value),
               'enabled' if self._loop.tools_enabled else 'DISABLED'))

    # --- callbacks --------------------------------------------------------

    def transcript_callback(self, msg):
        text = msg.text.strip()
        if msg.stop_word or (text and intent_schema.contains_stop_word(text)):
            self._barge_in()
            return
        if not text:
            self._silent_turn()
            return
        if not self._session_active:
            self._session_active = True
            self._history.clear()  # each conversation starts fresh
            self.get_logger().info('chat session started')
        self._silent_turns = 0
        self._queue.put(text)

    def speaking_callback(self, msg):
        self._speaking = bool(msg.data)
        if self._speaking:
            self._cancel_relisten_timer()
        elif self._awaiting_relisten:
            self._schedule_relisten()

    def voltage_callback(self, msg):
        self._voltage = float(msg.data)

    def motion_active_callback(self, msg):
        self._motion_active = bool(msg.data)

    # --- turn-taking ------------------------------------------------------

    def _barge_in(self):
        """Stop word heard: estop, drop the in-flight reply, silence the mouth.

        Runs entirely outside the LLM loop. The ear already called
        behavior/estop; calling it again is harmless and covers the case
        where the ear's client was not ready."""
        self._generation += 1
        self._drain_queue()
        self._do_estop(say_it=True)
        self.get_logger().info('barge-in: estop sent, reply abandoned, mouth flushed')
        if self._session_active:
            self._awaiting_relisten = True

    def _silent_turn(self):
        if not self._session_active:
            return
        self._silent_turns += 1
        limit = int(self.get_parameter('max_silent_turns').value)
        if self._silent_turns >= limit:
            self._say('Ending chat.', key='chat_timeout')
            self._end_session('%d silent turns' % self._silent_turns)
        else:
            self.get_logger().info('heard nothing (%d/%d) — listening again'
                                   % (self._silent_turns, limit))
            self._awaiting_relisten = True
            self._schedule_relisten()

    def _end_session(self, reason):
        self._session_active = False
        self._awaiting_relisten = False
        self._silent_turns = 0
        self.dialog.cancel()
        self._cancel_relisten_timer()
        self._sweep_recordings()
        self.get_logger().info('chat session ended (%s)' % reason)

    def _schedule_relisten(self):
        if not bool(self.get_parameter('auto_listen').value):
            return
        with self._lock:
            self._cancel_relisten_timer()
            delay = float(self.get_parameter('relisten_delay_s').value)
            self._relisten_timer = threading.Timer(delay, self._fire_relisten)
            self._relisten_timer.daemon = True
            self._relisten_timer.start()

    def _cancel_relisten_timer(self):
        if self._relisten_timer is not None:
            self._relisten_timer.cancel()
            self._relisten_timer = None

    def _fire_relisten(self):
        if not self._session_active or self._speaking:
            return  # a False transition re-schedules when the mouth finishes
        self._awaiting_relisten = False
        self.listen_pub.publish(Empty())

    def _drain_queue(self):
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                return

    # --- LLM worker -------------------------------------------------------

    def _worker_loop(self):
        while rclpy.ok():
            try:
                text = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._handle_turn(text)
            except Exception as e:
                self.get_logger().error('chat turn failed: %s' % e)
                self._say_key('chat_error')
                self._finish_turn(self._generation)
            finally:
                self._queue.task_done()

    def _handle_turn(self, text):
        if is_end_phrase(text):
            self._say('Goodbye.', key='chat_goodbye')
            self._end_session('goodbye phrase')
            return

        generation = self._generation

        # A pending go_to_point confirmation eats yes/no; anything else
        # implicitly cancels it and is processed normally.
        if self._consume_confirmation(text):
            self._finish_turn(generation)
            return

        try:
            self._loop.run(
                text,
                stream_fn=self._stream_chat,
                speak=lambda s: self._say(s, key='chat_reply'),
                run_tools=self._run_tools,
                still_current=lambda: generation == self._generation)
        except TurnFailed as e:
            self.get_logger().error('LLM unreachable/failed: %s' % e.cause)
            if e.spoke_any or not bool(self.get_parameter('offline_fallback').value):
                self._say_key('chat_error')
            else:
                self._offline_fallback(text)

        self._finish_turn(generation)

    def _finish_turn(self, generation):
        if generation != self._generation:
            return  # barge-in already re-armed the ear
        self._awaiting_relisten = True
        if not self._speaking:
            # Reply was empty, or the mouth already finished: don't wait
            # for a speaking transition that may never come.
            self._schedule_relisten()

    def _consume_confirmation(self, text):
        """True if the utterance was consumed as a yes/no to a pending goal."""
        pending = self.dialog.pending()
        if pending is None:
            return False
        parsed = self.rule_brain.parse(text).get('intent')
        if parsed == 'affirm':
            confirmed = self.dialog.resolve(True)
            self._history.add_user(text)
            self._history.add_assistant(
                self._say_key('nav_confirmed', **confirmed.params))
            self._send_behavior_later(confirmed)
            return True
        if parsed == 'deny':
            self.dialog.resolve(False)
            self._history.add_user(text)
            self._history.add_assistant(self._say_key('nav_cancelled'))
            return True
        self.dialog.cancel()
        return False

    def _stream_chat(self, messages, tool_schemas):
        """Stream one Ollama /api/chat completion, yielding chunk dicts."""
        try:
            import requests
        except ImportError as e:
            raise RuntimeError(
                'python3-requests missing (%s) — rebuild the image' % e)

        url = str(self.get_parameter('ollama_url').value).rstrip('/') + '/api/chat'
        payload = {
            'model': str(self.get_parameter('ollama_model').value),
            'messages': messages,
            'stream': True,
            'think': False,
            'keep_alive': str(self.get_parameter('keep_alive').value),
            'options': {
                'temperature': float(self.get_parameter('temperature').value),
                'num_predict': int(self.get_parameter('max_reply_tokens').value),
            },
        }
        if tool_schemas:
            payload['tools'] = tool_schemas
        timeout = (float(self.get_parameter('connect_timeout_s').value),
                   float(self.get_parameter('request_timeout_s').value))
        with requests.post(url, json=payload, stream=True,
                           timeout=timeout) as resp:
            resp.raise_for_status()
            for line in resp.iter_lines():
                if not line:
                    continue
                yield json.loads(line)

    # --- tool dispatch ----------------------------------------------------

    def _run_tools(self, calls, spoke_prose):
        """Validate one round of proposed calls (only the first is honoured,
        the rest are refused as compound) and dispatch them. Returns the
        result texts the model narrates. Called from the worker thread."""
        results = []
        for call, validated in zip(calls, tools.validate_round(calls)):
            self.get_logger().info('tool call %s(%r) -> %s%s' % (
                call.name, call.arguments, validated.intent.name,
                ' REJECTED: %s' % validated.intent.rejected_reason
                if validated.rejected else ''))
            results.append(self._dispatch(validated, spoke_prose))
        return results

    def _dispatch(self, validated, spoke_prose):
        intent = validated.intent
        for note in intent.clamp_notes:
            self.get_logger().warn('clamped: %s' % note)

        if validated.rejected:
            self._say_key(validated.ack_key)
            return validated.result_text

        if intent.is_stop:
            self._do_estop(say_it=True)
            return validated.result_text

        if intent.name in ('led_on', 'led_off', 'led_blink'):
            self._do_led(intent.name)
            return validated.result_text

        if intent.name == 'battery_status':
            if self._voltage is None:
                return 'Battery voltage is not available right now.'
            return 'Battery is at %.1f volts.' % self._voltage

        if intent.name == 'record_replay':
            return self._do_record_replay(validated)

        if intent.behavior_json is None:
            # Speech-only intents can't come from a tool; belt and braces.
            return validated.result_text

        # Motion / nav from here on.
        if not self.rate_limiter.allow():
            self.get_logger().warn('motion rate limit hit')
            self._say_key('rate_limited')
            return 'Refused: too many motion commands this minute.'

        if not self.behavior_client.server_is_ready():
            self.get_logger().error('behavior action server not available')
            self._say_key('not_ready')
            return 'Refused: the motion system is not responding.'

        if intent.requires_confirmation:
            self.dialog.request_confirmation(intent)
            self._say_key(validated.ack_key, **intent.params)
            return validated.result_text

        # Speak-then-act: the model's own sentence is the acknowledgement
        # unless the request was clamped (then the template states the
        # real distance) or the model said nothing (then the template is
        # the only audible warning before wheels move).
        if validated.ack_key and (intent.clamp_notes or not spoke_prose):
            self._say_key(validated.ack_key, **intent.params)
        self._send_behavior_later(intent)
        return validated.result_text

    def _offline_fallback(self, text):
        """LLM down: RuleBrain -> same validator -> same dispatch."""
        self._say_key('chat_offline')
        raw = self.rule_brain.parse(text)
        intent = intent_schema.validate(raw)
        self.get_logger().warn('offline fallback: %r -> %s' % (text, intent.name))
        if intent.behavior_json is None and not intent.is_stop:
            # Speech-only or unknown: the scripted line is all we have.
            if intent.name in ('led_on', 'led_off', 'led_blink'):
                self._do_led(intent.name)
                self._say_key(intent.reply_key)
            elif intent.name == 'battery_status':
                if self._voltage is None:
                    self._say_key('battery_status_unknown')
                else:
                    self._say_key('battery_status', voltage=self._voltage)
            else:
                self._say_key(intent.reply_key, **intent.params)
            return
        validated = tools.ValidatedTool(tool=intent.name, intent=intent,
                                        result_text='')
        # spoke_prose=False forces the scripted ack before any motion.
        self._dispatch(validated, spoke_prose=False)

    # --- record_replay ----------------------------------------------------

    def _do_record_replay(self, validated):
        """Countdown -> mouth silent -> ear records -> playbacks queued.
        Runs on the worker thread; blocks it for the recording, which is
        fine: nothing else may happen while the mic is recording anyway."""
        p = validated.intent.params
        duration, speeds = p['duration_s'], p['speeds']
        if self._motion_busy():
            self.get_logger().warn('record_replay refused: motion active')
            self._say_key('record_refused_moving')
            return 'Refused: the robot is moving; recording is not allowed while moving.'
        if not self.record_client.service_is_ready():
            self.get_logger().error('voice/record service not available')
            self._say_key('record_unavailable')
            return 'Refused: the microphone recording service is not available.'

        os.makedirs(self.recordings_dir, exist_ok=True)
        path = os.path.join(self.recordings_dir, 'rec_%d.wav' % int(time.time() * 1000))

        self._say_key('record_countdown', duration_s=duration)
        self._wait_mouth_idle(timeout=20.0)
        if self._motion_busy():   # a goal landed meanwhile
            self._say_key('record_refused_moving')
            return 'Refused: the robot started moving.'

        req = Record.Request()
        req.duration_s = float(duration)
        req.path = path
        future = self.record_client.call_async(req)
        deadline = time.monotonic() + duration + 15.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        result = future.result() if future.done() else None
        if result is None or not result.success:
            self.get_logger().error('record failed: %s' % (
                result.message if result else 'timeout'))
            self._say_key('record_failed')
            self._unlink_quiet(path)
            return 'Refused: recording failed (%s).' % (
                result.message if result else 'timeout')

        self._say_key('record_playback')
        for i, speed in enumerate(speeds):
            msg = Say()
            msg.key = 'record_playback_wav'
            msg.wav_path = result.path
            msg.speed_factor = float(speed)
            msg.delete_after = (i == len(speeds) - 1)   # never persists
            self.say_pub.publish(msg)
        self.get_logger().info('record_replay: %.1f s recorded, playbacks at %s'
                               % (duration, speeds))
        return validated.result_text

    def _motion_busy(self):
        """True while anything moves, a goal waits in the speak-then-act
        window, or a goal was sent so recently that behavior_ctrl's latched
        flag may not have caught up yet."""
        with self._lock:
            pending = self._pending_goals
            recent = time.monotonic() - self._last_goal_sent < self.MOTION_FLAG_GRACE_S
        return self._motion_active or pending > 0 or recent

    def _wait_mouth_idle(self, timeout):
        """Wait for the countdown to start playing and then finish, so the
        recording does not capture the robot's own voice."""
        t0 = time.monotonic()
        while not self._speaking and time.monotonic() - t0 < 3.0:
            time.sleep(0.05)
        while self._speaking and time.monotonic() - t0 < timeout:
            time.sleep(0.05)

    def _sweep_recordings(self):
        """Recordings are temp files; make sure none outlive a session."""
        for f in glob.glob(os.path.join(self.recordings_dir, 'rec_*.wav')):
            self._unlink_quiet(f)

    @staticmethod
    def _unlink_quiet(path):
        try:
            os.unlink(path)
        except OSError:
            pass

    # --- motion dispatch --------------------------------------------------

    def _send_behavior_later(self, intent):
        """Speak-then-act: delay the goal so the ack is audible before wheels
        move, but abort silently if a stop lands during the delay."""
        with self._lock:
            gen = self._stop_gen
            self._pending_goals += 1

        def send():
            with self._lock:
                self._pending_goals -= 1
                if gen != self._stop_gen:
                    self.get_logger().warn(
                        '%s: dropped — stop requested during speak-then-act '
                        'delay' % intent.name)
                    return
                self._last_goal_sent = time.monotonic()
            goal = Behavior.Goal()
            goal.command = intent.behavior_json
            self.get_logger().info('sending behavior goal: %s' % intent.behavior_json)
            future = self.behavior_client.send_goal_async(goal)
            future.add_done_callback(self._goal_response)

        timer = threading.Timer(self.speak_before_act_s, send)
        timer.daemon = True
        timer.start()

    def _goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('behavior goal rejected')

    def _do_estop(self, say_it):
        with self._lock:
            self._stop_gen += 1
        self.dialog.cancel()
        if say_it:
            self._say_key('estop', priority=Say.PRIORITY_SAFETY)
        if self.estop_client.service_is_ready():
            self.estop_client.call_async(Trigger.Request())
        else:
            self.get_logger().error('behavior/estop service not available!')

    def _do_led(self, name):
        if name == 'led_blink':
            # Three cycles, driven by timers so nothing blocks.
            for i in range(6):
                level = 255.0 if i % 2 == 0 else 0.0
                t = threading.Timer(0.4 * i, self._publish_led, args=(level,))
                t.daemon = True
                t.start()
        else:
            self._publish_led(255.0 if name == 'led_on' else 0.0)

    def _publish_led(self, level):
        msg = Float32MultiArray()
        msg.data = [level, level]  # IO4, IO5
        self.led_pub.publish(msg)

    # --- output -----------------------------------------------------------

    def _say(self, text, priority=Say.PRIORITY_NORMAL, key=''):
        msg = Say()
        msg.key = key
        msg.text = text
        msg.priority = priority
        self.say_pub.publish(msg)
        self.get_logger().info('say[%s]: %s' % (key, text))

    def _say_key(self, key, priority=Say.PRIORITY_NORMAL, **slots):
        """Speak a scripted responses.py line (refusals, acks, safety)."""
        text = responses.render(key, **slots)
        self._say(text, priority=priority, key=key)
        return text


def main(args=None):
    rclpy.init(args=args)
    node = ChatNode()
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
