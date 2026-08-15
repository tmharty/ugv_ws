"""voice_mouth: /voice/say -> TTS -> speaker.

Queued playback through a worker thread. PRIORITY_SAFETY messages flush the
queue and kill the current aplay so "Stopping." is never stuck behind chatter.
Publishes /voice/speaking (latched-ish, on every transition) so the ear can
mute ASR while the robot talks.

TTS backend: Piper (pleasantly synthetic, once its model is fetched) with
espeak-ng as the always-available fallback — the robot must never be mute.
Both render to a temp WAV and play via aplay on the configured ALSA device,
so interruption is one process kill either way.
"""

import os
import queue
import shutil
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

        self.playback_device = str(self.get_parameter('playback_device').value)

        self.speaking_pub = self.create_publisher(Bool, '/voice/speaking', 10)
        self.create_subscription(Say, '/voice/say', self.say_callback, 10)

        self._queue = queue.Queue()
        self._player = None            # current aplay Popen
        self._player_lock = threading.Lock()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self._publish_speaking(False)
        self.get_logger().info(
            f'voice_mouth up — backend: {self._backend()}, '
            f'device: {self.playback_device}')

    # --- queueing ---------------------------------------------------------

    def say_callback(self, msg):
        if msg.priority == Say.PRIORITY_SAFETY:
            self._flush_queue()
            self._kill_current()
        self._queue.put(msg)

    def _flush_queue(self):
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                return

    def _kill_current(self):
        with self._player_lock:
            if self._player is not None and self._player.poll() is None:
                self._player.kill()

    # --- playback worker --------------------------------------------------

    def _worker_loop(self):
        while True:
            msg = self._queue.get()
            if msg is None:
                break
            self._publish_speaking(True)
            try:
                self._speak(msg.text)
            except Exception as e:
                self.get_logger().error(f'TTS failed: {e}')
            finally:
                self._queue.task_done()
                if self._queue.empty():
                    self._publish_speaking(False)

    def _speak(self, text):
        if not text.strip():
            return
        wav = self._synthesize(text)
        if wav is None:
            return
        try:
            with self._player_lock:
                self._player = subprocess.Popen(
                    ['aplay', '-q', '-D', self.playback_device, wav],
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            _, err = self._player.communicate()
            if self._player.returncode not in (0, -9):  # -9 = interrupted, fine
                self.get_logger().error(
                    f'aplay failed on {self.playback_device}: '
                    f'{err.decode(errors="replace").strip()}')
        finally:
            os.unlink(wav)

    # --- synthesis --------------------------------------------------------

    def _backend(self):
        choice = str(self.get_parameter('tts_backend').value)
        piper_model = str(self.get_parameter('piper_model').value)
        piper_ok = shutil.which('piper') and os.path.exists(piper_model)
        if choice == 'piper' and not piper_ok:
            self.get_logger().warn(
                'tts_backend=piper but piper/model missing — using espeak-ng')
            return 'espeak'
        if choice == 'auto':
            return 'piper' if piper_ok else 'espeak'
        return choice

    def _synthesize(self, text):
        fd, wav = tempfile.mkstemp(suffix='.wav', prefix='ugv_say_')
        os.close(fd)
        if self._backend() == 'piper':
            cmd = ['piper', '--model',
                   str(self.get_parameter('piper_model').value),
                   '--output_file', wav]
            run = subprocess.run(cmd, input=text.encode(),
                                 capture_output=True, timeout=30)
        else:
            cmd = ['espeak-ng', '-v', str(self.get_parameter('espeak_voice').value),
                   '-s', str(int(self.get_parameter('espeak_speed').value)),
                   '-w', wav, text]
            run = subprocess.run(cmd, capture_output=True, timeout=30)
        if run.returncode != 0:
            self.get_logger().error(
                f'{cmd[0]} failed: {run.stderr.decode(errors="replace").strip()}')
            os.unlink(wav)
            return None
        return wav

    def _publish_speaking(self, value):
        self.speaking_pub.publish(Bool(data=value))

    def destroy_node(self):
        self._queue.put(None)
        self._kill_current()
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
