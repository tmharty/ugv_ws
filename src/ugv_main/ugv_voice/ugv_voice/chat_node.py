"""voice_chat: /voice/transcript -> Ollama LLM -> /voice/say.

A conversational companion deliberately OUTSIDE the control path: no
Behavior action client, no /cmd_vel, no estop wiring — this node can talk
and do nothing else. It reuses the existing ear (mic/ASR) and mouth (TTS)
nodes and talks to the Ollama daemon on the Jetson host (localhost works
inside the container because run_jetson.sh uses --network host).

Session flow: publish /voice/listen_once and speak. Each reply is streamed
from the LLM sentence-by-sentence into /voice/say so speech starts before
the model finishes writing. When the mouth falls silent the node re-arms
the ear itself (auto_listen), so one trigger starts a whole conversation.
The session ends on a goodbye phrase, after max_silent_turns empty
listens, or if the LLM is unreachable.

A stop-word transcript barges in: the in-flight reply is abandoned and the
mouth's queue is flushed via a PRIORITY_SAFETY message (the ear will also
have called behavior/estop, which simply isn't running in chat-only mode).
"""

import json
import queue
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from ugv_interface.msg import Say, Transcript

from .chat_session import (ChatHistory, DEFAULT_SYSTEM_PROMPT,
                           SentenceChunker, is_end_phrase)


class ChatNode(Node):
    def __init__(self):
        super().__init__('voice_chat')

        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('ollama_model', 'gemma3n:e2b')
        # First request includes model load on the Jetson — allow for it.
        self.declare_parameter('request_timeout_s', 120.0)
        self.declare_parameter('keep_alive', '10m')
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_reply_tokens', 120)
        self.declare_parameter('max_history_turns', 12)
        self.declare_parameter('system_prompt', '')  # '' = built-in default
        self.declare_parameter('auto_listen', True)
        self.declare_parameter('relisten_delay_s', 0.7)
        self.declare_parameter('max_silent_turns', 2)

        system_prompt = (str(self.get_parameter('system_prompt').value)
                         or DEFAULT_SYSTEM_PROMPT)
        self._history = ChatHistory(
            system_prompt,
            max_turns=int(self.get_parameter('max_history_turns').value))

        self.say_pub = self.create_publisher(Say, '/voice/say', 10)
        self.listen_pub = self.create_publisher(Empty, '/voice/listen_once', 10)
        self.create_subscription(Transcript, '/voice/transcript',
                                 self.transcript_callback, 10)
        self.create_subscription(Bool, '/voice/speaking',
                                 self.speaking_callback, 10)

        self._queue = queue.Queue()
        self._generation = 0        # bumped on barge-in; worker drops stale work
        self._speaking = False
        self._session_active = False
        self._silent_turns = 0
        self._awaiting_relisten = False
        self._relisten_timer = None
        self._lock = threading.Lock()

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'voice_chat up — model %r at %s (talk-only: no control interfaces).'
            ' Publish /voice/listen_once to start a conversation.'
            % (str(self.get_parameter('ollama_model').value),
               str(self.get_parameter('ollama_url').value)))

    # --- callbacks --------------------------------------------------------

    def transcript_callback(self, msg):
        text = msg.text.strip()
        if msg.stop_word:
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

    # --- turn-taking ------------------------------------------------------

    def _barge_in(self):
        """Stop word heard: drop the in-flight reply and silence the mouth."""
        self._generation += 1
        self._drain_queue()
        self.get_logger().info('barge-in: reply abandoned, mouth flushed')
        if self._session_active:
            self._say('Okay.', priority=Say.PRIORITY_SAFETY, key='chat_barge_in')
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
        self._cancel_relisten_timer()
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
                self._say('My chat brain is not answering. Ending chat.',
                          key='chat_error')
                self._end_session('LLM failure')
            finally:
                self._queue.task_done()

    def _handle_turn(self, text):
        if is_end_phrase(text):
            self._say('Goodbye.', key='chat_goodbye')
            self._end_session('goodbye phrase')
            return

        generation = self._generation
        self._history.add_user(text)
        spoken = []
        for sentence in self._iter_reply_sentences(self._history.messages()):
            if generation != self._generation:
                break  # barge-in mid-stream
            self._say(sentence, key='chat_reply')
            spoken.append(sentence)
        self._history.add_assistant(' '.join(spoken))

        if generation == self._generation:
            self._awaiting_relisten = True
            if not self._speaking:
                # Reply was empty, or the mouth already finished: don't wait
                # for a speaking transition that may never come.
                self._schedule_relisten()

    def _iter_reply_sentences(self, messages):
        """Stream the Ollama chat completion, yielding whole sentences."""
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
        chunker = SentenceChunker()
        timeout = (5.0, float(self.get_parameter('request_timeout_s').value))
        with requests.post(url, json=payload, stream=True,
                           timeout=timeout) as resp:
            resp.raise_for_status()
            for line in resp.iter_lines():
                if not line:
                    continue
                data = json.loads(line)
                if data.get('done'):
                    break
                for sentence in chunker.feed(
                        data.get('message', {}).get('content', '')):
                    yield sentence
        tail = chunker.flush()
        if tail:
            yield tail

    # --- output -----------------------------------------------------------

    def _say(self, text, priority=Say.PRIORITY_NORMAL, key=''):
        msg = Say()
        msg.key = key
        msg.text = text
        msg.priority = priority
        self.say_pub.publish(msg)


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
