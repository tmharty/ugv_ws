"""voice_ear: microphone -> transcript.

Phase 1 operation is push-to-talk: publish anything on /voice/listen_once and
the node records one utterance (energy-endpointed, hard-capped listen window),
runs faster-whisper ASR on it, and publishes a Transcript. Wake-word gating is
behind the wake_word_enabled param and becomes the default once PTT is proven.

Safety path: every transcript is scanned for the stop lexicon HERE, and a hit
calls behavior/estop directly — the brain node is not in that loop.

Heavy deps (sounddevice, faster-whisper) are imported lazily with loud, clear
log messages when missing, so the node still starts on an image that hasn't
been rebuilt yet.
"""

import math
import os
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
from ugv_interface.msg import Transcript

from .intent_schema import contains_stop_word


class EarNode(Node):
    def __init__(self):
        super().__init__('voice_ear')

        # Device is a PortAudio name substring (e.g. 'Camera'), resolved at
        # capture time — ALSA card indices shift across boots, names don't.
        self.declare_parameter('capture_device', 'Camera')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('listen_window_s', 6.0)
        self.declare_parameter('silence_after_speech_s', 0.8)
        self.declare_parameter('energy_threshold', 0.015)  # RMS, 0..1 scale
        self.declare_parameter('asr_model', 'base.en')
        self.declare_parameter('models_dir', '/home/ws/ugv_ws/models')
        self.declare_parameter('wake_word_enabled', False)
        self.declare_parameter('wake_word_model', 'hey_jarvis')
        self.declare_parameter('wake_word_threshold', 0.6)

        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.listen_window_s = float(self.get_parameter('listen_window_s').value)
        self.silence_s = float(self.get_parameter('silence_after_speech_s').value)
        self.energy_threshold = float(self.get_parameter('energy_threshold').value)

        self.transcript_pub = self.create_publisher(Transcript, '/voice/transcript', 10)
        self.estop_pub = self.create_publisher(Empty, '/voice/estop', 10)
        self.estop_client = self.create_client(Trigger, 'behavior/estop')

        self.create_subscription(Empty, '/voice/listen_once', self.listen_once_callback, 10)
        self.create_subscription(Bool, '/voice/speaking', self.speaking_callback, 10)

        self._speaking = False
        self._listen_requested = threading.Event()
        self._asr = None
        self._asr_label = 'none'

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        mode = ('wake word' if bool(self.get_parameter('wake_word_enabled').value)
                else 'push-to-talk (/voice/listen_once)')
        self.get_logger().info(f'voice_ear up — mode: {mode}')

    # --- callbacks --------------------------------------------------------

    def speaking_callback(self, msg):
        self._speaking = bool(msg.data)

    def listen_once_callback(self, _msg):
        if self._speaking:
            self.get_logger().info('listen_once ignored: robot is speaking')
            return
        if self._listen_requested.is_set():
            self.get_logger().info('listen_once ignored: already listening')
            return
        self._listen_requested.set()

    # --- capture ----------------------------------------------------------

    def _worker_loop(self):
        wake_enabled = bool(self.get_parameter('wake_word_enabled').value)
        while rclpy.ok():
            # Everything in the cycle is guarded: an exception (bad device,
            # broken wake stack, model load failure) must never kill this
            # thread — the ear just logs and keeps listening.
            try:
                if wake_enabled:
                    if not self._wait_for_wake_word():
                        continue
                elif not self._listen_requested.wait(timeout=0.5):
                    continue
                audio = self._record_utterance()
                if audio is not None:
                    self._transcribe_and_publish(audio)
            except Exception as e:
                self.get_logger().error(f'listen cycle failed: {e}')
            finally:
                self._listen_requested.clear()

    def _record_utterance(self):
        """Record until trailing silence after speech, capped at the listen
        window. Returns float32 mono audio at sample_rate, or None."""
        try:
            import numpy as np
            import sounddevice as sd
        except ImportError as e:
            self.get_logger().error(
                f'audio capture unavailable ({e}) — rebuild the image with '
                'the Phase 0/1 audio deps (sounddevice)')
            return None

        device = self._resolve_device(sd)
        max_blocks = int(self.listen_window_s / 0.03)
        silence_blocks = max(1, int(self.silence_s / 0.03))

        chunks = []
        speech_started = False
        quiet_run = 0
        self.get_logger().info('listening…')
        try:
            stream, capture_rate = self._open_input_stream(sd, device,
                                                           'float32', 0.03)
            block = int(capture_rate * 0.03)
            with stream:
                for _ in range(max_blocks):
                    data, _overflow = stream.read(block)
                    mono = data[:, 0]
                    chunks.append(mono.copy())
                    rms = float(np.sqrt(np.mean(mono ** 2)))
                    if rms >= self.energy_threshold:
                        speech_started = True
                        quiet_run = 0
                    elif speech_started:
                        quiet_run += 1
                        if quiet_run >= silence_blocks:
                            break
        except Exception as e:
            self.get_logger().error(
                f'mic capture failed on device {device!r}: {e} — devices: '
                f'{[d["name"] for d in sd.query_devices()]}')
            return None

        if not speech_started:
            self._publish_transcript('', 0.0)  # brain answers "heard nothing"
            return None
        return self._resample(np, np.concatenate(chunks),
                              capture_rate, self.sample_rate)

    def _resolve_device(self, sd):
        name = str(self.get_parameter('capture_device').value)
        try:
            return sd.query_devices(name, kind='input')['index']
        except ValueError:
            self.get_logger().warn(
                f'capture_device {name!r} not found — using system default')
            return None

    def _open_input_stream(self, sd, device, dtype, block_s):
        """Open a mono input stream at sample_rate; if the hardware refuses
        (raw hw: ALSA devices do no rate conversion — USB webcam mics are
        often 44.1/48 kHz only), reopen at the device's native rate. Returns
        (unstarted stream, actual capture rate); callers resample to
        sample_rate."""
        try:
            block = int(self.sample_rate * block_s)
            return (sd.InputStream(device=device, channels=1, dtype=dtype,
                                   samplerate=self.sample_rate,
                                   blocksize=block),
                    self.sample_rate)
        except sd.PortAudioError:
            native = int(sd.query_devices(device, kind='input')
                         ['default_samplerate'])
            self.get_logger().warn(
                f'device {device!r} cannot capture at {self.sample_rate} Hz '
                f'— capturing at {native} Hz and resampling')
            block = int(native * block_s)
            return (sd.InputStream(device=device, channels=1, dtype=dtype,
                                   samplerate=native, blocksize=block),
                    native)

    @staticmethod
    def _resample(np, audio, from_rate, to_rate):
        """Linear-interpolation resample — adequate for speech ASR/wake."""
        if from_rate == to_rate:
            return audio
        n = int(round(len(audio) * to_rate / from_rate))
        resampled = np.interp(np.linspace(0.0, len(audio) - 1, n),
                              np.arange(len(audio)), audio)
        return resampled.astype(audio.dtype)

    # --- ASR --------------------------------------------------------------

    def _load_asr(self):
        if self._asr is not None:
            return self._asr
        model_name = str(self.get_parameter('asr_model').value)
        models_dir = str(self.get_parameter('models_dir').value)
        try:
            from faster_whisper import WhisperModel
        except ImportError:
            self.get_logger().error(
                'faster-whisper not installed — rebuild the image with the '
                'Phase 1 voice deps; publishing empty transcript')
            return None
        # Prefer the directory scripts/fetch_voice_models.sh populates, so the
        # robot works offline; fall back to a by-name Hugging Face download.
        local_dir = os.path.join(models_dir, f'faster-whisper-{model_name}')
        model_ref = local_dir if os.path.isdir(local_dir) else model_name
        self._asr = WhisperModel(model_ref, device='cpu', compute_type='int8',
                                 download_root=models_dir)
        self._asr_label = f'faster-whisper-{model_name}-int8'
        self.get_logger().info(f'loaded ASR: {self._asr_label}')
        return self._asr

    def _transcribe_and_publish(self, audio):
        asr = self._load_asr()
        if asr is None:
            return
        segments, _info = asr.transcribe(audio, language='en', beam_size=1,
                                         vad_filter=True)
        texts, confidence = [], 0.0
        for seg in segments:
            texts.append(seg.text.strip())
            confidence = max(confidence,
                             math.exp(min(0.0, seg.avg_logprob)))
        text = ' '.join(t for t in texts if t)
        self.get_logger().info(f'heard: {text!r} (conf {confidence:.2f})')
        self._publish_transcript(text, confidence)

    def _publish_transcript(self, text, confidence):
        stop_hit = bool(text) and contains_stop_word(text)
        if stop_hit:
            self._fire_estop()

        msg = Transcript()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.text = text
        msg.confidence = float(confidence)
        msg.asr_backend = self._asr_label
        msg.stop_word = stop_hit
        self.transcript_pub.publish(msg)

    def _fire_estop(self):
        """Stop path independent of the brain: service call + observable topic."""
        self.get_logger().warn('stop word heard — calling behavior/estop')
        self.estop_pub.publish(Empty())
        if self.estop_client.service_is_ready():
            self.estop_client.call_async(Trigger.Request())
        else:
            self.get_logger().error('behavior/estop service not available!')

    # --- wake word (param-gated; default off until PTT is proven) ---------

    def _resolve_wake_model(self):
        """Map the wake_word_model param to the local file fetched by
        scripts/fetch_voice_models.sh, falling back to the bare name (which
        openwakeword resolves from its own package resources)."""
        name = str(self.get_parameter('wake_word_model').value)
        for candidate in (name, f'{name}.onnx', f'{name}_v0.1.onnx'):
            path = os.path.join(self._oww_dir(), candidate)
            if os.path.isfile(path):
                return path
        return name

    def _oww_dir(self):
        return os.path.join(str(self.get_parameter('models_dir').value),
                            'openwakeword')

    def _oww_feature_paths(self):
        """openwakeword's shared feature models, if fetched locally — without
        these kwargs it looks in its package resources, which are empty unless
        openwakeword.utils.download_models() was run."""
        paths = {'melspec_model_path': 'melspectrogram.onnx',
                 'embedding_model_path': 'embedding_model.onnx'}
        resolved = {k: os.path.join(self._oww_dir(), f)
                    for k, f in paths.items()}
        return resolved if all(os.path.isfile(p) for p in resolved.values()) else {}

    def _wait_for_wake_word(self):
        try:
            import numpy as np
            import sounddevice as sd
            from openwakeword.model import Model as WakeModel
        except ImportError as e:
            self.get_logger().error(
                f'wake word unavailable ({e}) — falling back to push-to-talk')
            self._listen_requested.wait()
            return True

        if not hasattr(self, '_wake_model'):
            self._wake_model = WakeModel(
                wakeword_models=[self._resolve_wake_model()],
                inference_framework='onnx',
                **self._oww_feature_paths())
        threshold = float(self.get_parameter('wake_word_threshold').value)
        device = self._resolve_device(sd)
        # openwakeword expects 80 ms frames at sample_rate (16 kHz)
        stream, capture_rate = self._open_input_stream(sd, device,
                                                       'int16', 0.08)
        block = int(capture_rate * 0.08)

        with stream:
            while rclpy.ok():
                if self._listen_requested.is_set():
                    return True  # PTT still works in wake-word mode
                data, _ = stream.read(block)
                if self._speaking:
                    continue  # don't wake on our own voice
                frame = self._resample(np, np.squeeze(data),
                                       capture_rate, self.sample_rate)
                scores = self._wake_model.predict(frame)
                if any(s >= threshold for s in scores.values()):
                    self.get_logger().info('wake word detected')
                    self._wake_model.reset()
                    return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = EarNode()
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
