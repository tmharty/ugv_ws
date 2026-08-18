"""voice_mouth: /voice/say -> TTS -> speaker.

Two-stage pipeline: a synth thread turns queued sentences into audio while a
playback thread plays the previous one, so the gap between sentences is one
aplay spawn (~10 ms) instead of a full Piper cold start. The Piper voice is
loaded once at startup through its Python API — reloading the ONNX model per
sentence was the dominant source of inter-sentence dead air. espeak-ng stays
as the always-available fallback: the robot must never be mute.

/voice/speaking is True from the moment a reply's first sentence is accepted
until speaking_grace_s after the last queued audio finishes playing, so a
brief gap between sentences can't flap it False and re-arm the ear mid-reply.

PRIORITY_SAFETY messages flush both queues and kill the current aplay so
"Stopping." is never stuck behind chatter.
"""

import os
import queue
import subprocess
import tempfile
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from ugv_interface.msg import Say


class MouthNode(Node):
    def __init__(self):
        super().__init__('voice_mouth')

        self.declare_parameter('playback_device', 'plughw:CARD=Device,DEV=0')
        self.declare_parameter('tts_backend', 'auto')  # auto | piper | espeak
        self.declare_parameter('piper_model',
                               '/home/ws/ugv_ws/models/piper/en_US-lessac-medium.onnx')
        self.declare_parameter('espeak_voice', 'en-us')
        self.declare_parameter('espeak_speed', 155)
        self.declare_parameter('speaking_grace_s', 0.4)

        self.playback_device = str(self.get_parameter('playback_device').value)
        self.speaking_grace_s = float(
            self.get_parameter('speaking_grace_s').value)

        self.speaking_pub = self.create_publisher(Bool, '/voice/speaking', 10)
        self.create_subscription(Say, '/voice/say', self.say_callback, 10)

        self._piper = None
        self._piper_old_api = False
        self._piper_rate = 0
        self._tts_backend = self._init_backend()

        # Pipeline state. Invariant: every message accepted by say_callback
        # is finished exactly once via _finish_item — after playback, on
        # synth failure, on a stale-epoch drop, or when drained by a safety
        # flush. That is what keeps /voice/speaking truthful.
        self._state_lock = threading.Lock()
        self._epoch = 0                # bumped by safety flush; stale items drop
        self._pending = 0              # accepted but not yet finished messages
        self._speaking = False
        self._grace_timer = None
        self._text_queue = queue.Queue()
        self._audio_queue = queue.Queue(maxsize=2)  # bounds flush latency/memory
        self._player = None            # current aplay Popen
        self._player_lock = threading.Lock()
        self._synth_thread = threading.Thread(target=self._synth_loop,
                                              daemon=True)
        self._playback_thread = threading.Thread(target=self._playback_loop,
                                                 daemon=True)
        self._synth_thread.start()
        self._playback_thread.start()

        self._publish_speaking(False)
        self.get_logger().info(
            f'voice_mouth up — backend: {self._tts_backend}, '
            f'device: {self.playback_device}')

    # --- queueing -----------------------------------------------------------

    def say_callback(self, msg):
        if msg.priority == Say.PRIORITY_SAFETY:
            self._flush_pipeline()
            self._kill_current()
        if not msg.text.strip():
            return
        with self._state_lock:
            if self._grace_timer is not None:
                self._grace_timer.cancel()
                self._grace_timer = None
            self._pending += 1
            announce = not self._speaking
            self._speaking = True
            epoch = self._epoch
        if announce:
            self._publish_speaking(True)
        self._text_queue.put((epoch, msg.text))

    def _flush_pipeline(self):
        """Safety flush: invalidate everything queued in both stages so only
        the incoming safety message plays. The item currently being
        synthesized or played is handled by its own thread (stale drop /
        process kill), never here — so each item still finishes exactly once."""
        with self._state_lock:
            self._epoch += 1
        for q in (self._text_queue, self._audio_queue):
            while True:
                try:
                    entry = q.get_nowait()
                except queue.Empty:
                    break
                if entry is None:      # shutdown sentinel — keep it in flight
                    q.put(None)
                    break
                if q is self._audio_queue:
                    self._discard_audio(entry[1])
                self._finish_item()

    def _kill_current(self):
        with self._player_lock:
            if self._player is not None and self._player.poll() is None:
                self._player.kill()

    # --- pipeline threads ---------------------------------------------------

    def _synth_loop(self):
        while True:
            entry = self._text_queue.get()
            if entry is None:
                self._audio_queue.put(None)
                return
            epoch, text = entry
            if self._stale(epoch):
                self._finish_item()
                continue
            item = None
            try:
                item = self._synthesize(text)
            except Exception as e:
                self.get_logger().error(f'TTS failed: {e}')
            if item is None:
                self._finish_item()
                continue
            # Bounded put with a timeout so a safety flush can never leave
            # this thread parked on a queue nobody is draining.
            while True:
                if self._stale(epoch):
                    self._discard_audio(item)
                    self._finish_item()
                    break
                try:
                    self._audio_queue.put((epoch, item), timeout=0.2)
                    break
                except queue.Full:
                    continue

    def _playback_loop(self):
        while True:
            entry = self._audio_queue.get()
            if entry is None:
                return
            epoch, item = entry
            if self._stale(epoch):
                self._discard_audio(item)
            else:
                try:
                    self._play(item)
                except Exception as e:
                    self.get_logger().error(f'playback failed: {e}')
            self._finish_item()

    def _stale(self, epoch):
        with self._state_lock:
            return epoch != self._epoch

    def _finish_item(self):
        """Single decrement point for the pending counter. /voice/speaking
        only drops to False speaking_grace_s after the pipeline goes idle."""
        with self._state_lock:
            self._pending = max(0, self._pending - 1)
            if self._pending != 0:
                return
            if self._grace_timer is not None:
                self._grace_timer.cancel()
            self._grace_timer = threading.Timer(self.speaking_grace_s,
                                                self._grace_expired)
            self._grace_timer.daemon = True
            self._grace_timer.start()

    def _grace_expired(self):
        with self._state_lock:
            if self._pending != 0:
                return
            self._speaking = False
        self._publish_speaking(False)

    # --- synthesis ----------------------------------------------------------

    def _init_backend(self):
        """Resolve the TTS backend once and, for Piper, load the voice model
        now. Returns the backend name actually in use."""
        choice = str(self.get_parameter('tts_backend').value)
        if choice == 'espeak':
            return 'espeak'
        model_path = str(self.get_parameter('piper_model').value)
        try:
            try:
                from piper import PiperVoice
            except ImportError:
                from piper.voice import PiperVoice  # older package layout
            if not os.path.exists(model_path):
                raise FileNotFoundError(f'no such model: {model_path}')
            self.get_logger().info(f'loading piper voice: {model_path}')
            self._piper = PiperVoice.load(model_path)
            # piper-tts <= 1.2 streams raw bytes; 1.3+ yields AudioChunks.
            self._piper_old_api = hasattr(self._piper, 'synthesize_stream_raw')
            self._piper_rate = int(self._piper.config.sample_rate)
            self.get_logger().info(f'piper voice loaded ({self._piper_rate} Hz)')
            return 'piper'
        except Exception as e:
            log = (self.get_logger().warn if choice == 'piper'
                   else self.get_logger().info)
            log(f'piper unavailable ({e}) — using espeak-ng')
            return 'espeak'

    def _synthesize(self, text):
        """Return ('pcm', bytes, rate) or ('wav', path, None), or None on
        failure. Piper renders to in-memory S16 mono PCM — no temp files."""
        if self._tts_backend == 'piper':
            if self._piper_old_api:
                pcm = b''.join(self._piper.synthesize_stream_raw(text))
            else:
                pcm = b''.join(chunk.audio_int16_bytes
                               for chunk in self._piper.synthesize(text))
            return ('pcm', pcm, self._piper_rate)

        fd, wav = tempfile.mkstemp(suffix='.wav', prefix='ugv_say_')
        os.close(fd)
        cmd = ['espeak-ng', '-v', str(self.get_parameter('espeak_voice').value),
               '-s', str(int(self.get_parameter('espeak_speed').value)),
               '-w', wav, text]
        run = subprocess.run(cmd, capture_output=True, timeout=30)
        if run.returncode != 0:
            self.get_logger().error(
                f'espeak-ng failed: {run.stderr.decode(errors="replace").strip()}')
            os.unlink(wav)
            return None
        return ('wav', wav, None)

    @staticmethod
    def _discard_audio(item):
        if item[0] == 'wav':
            try:
                os.unlink(item[1])
            except OSError:
                pass

    # --- playback -----------------------------------------------------------

    def _play(self, item):
        kind, payload, rate = item
        if kind == 'pcm':
            with self._player_lock:
                self._player = subprocess.Popen(
                    ['aplay', '-q', '-t', 'raw', '-f', 'S16_LE', '-c', '1',
                     '-r', str(rate), '-D', self.playback_device],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            try:
                _, err = self._player.communicate(input=payload)
            except (BrokenPipeError, OSError):
                return  # player killed mid-write (barge-in) — expected
            self._check_player(err)
        else:
            try:
                with self._player_lock:
                    self._player = subprocess.Popen(
                        ['aplay', '-q', '-D', self.playback_device, payload],
                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
                _, err = self._player.communicate()
                self._check_player(err)
            finally:
                os.unlink(payload)

    def _check_player(self, err):
        if self._player.returncode not in (0, -9):  # -9 = interrupted, fine
            self.get_logger().error(
                f'aplay failed on {self.playback_device}: '
                f'{err.decode(errors="replace").strip()}')

    def _publish_speaking(self, value):
        self.speaking_pub.publish(Bool(data=value))

    def destroy_node(self):
        self._text_queue.put(None)  # synth loop forwards it to the audio queue
        self._kill_current()
        with self._state_lock:
            if self._grace_timer is not None:
                self._grace_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MouthNode()
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
