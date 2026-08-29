"""voice_ear: microphone -> transcript, plus the always-on safety ear.

Listening (push-to-talk or wake word): one utterance is captured with
Silero VAD endpointing (energy gate fallback), transcribed by
faster-whisper, and published as a Transcript. /voice/listening goes True
for the capture window so the mouth can play a chime.

Monitor mode: whenever the wake word is enabled OR the robot is moving,
the worker keeps one input stream open and runs two detectors on every
80 ms frame — openWakeWord (when not speaking: half-duplex mute) and the
continuous stop watch. The stop watch segments speech with Silero and
transcribes each short segment straight away; a stop-lexicon hit calls
behavior/estop and publishes a stop_word Transcript so the chat node
barges in. No custom "stop" wake model is needed and nothing leaves the
device. stop_watch_while_speaking stays off until the mic array's AEC
arrives: without it the robot hears its own "Stopping.".

Safety path: every transcript is scanned for the stop lexicon HERE, and a
hit calls behavior/estop directly — no brain in that loop.

voice/record service (record_replay tool): captures N seconds of raw audio
to a wav on the worker thread — the thread that owns the microphone — so it
never fights the monitor stream for the device. The stop watch is
necessarily down while recording; callers refuse to record while moving.

Heavy deps (sounddevice, faster-whisper, openwakeword) are imported lazily
with loud, clear log messages when missing, so the node still starts on an
image that hasn't been rebuilt yet.
"""

import math
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger
from ugv_interface.msg import Transcript
from ugv_interface.srv import Record

from .intent_schema import contains_stop_word
from .vad import SILERO_RATE, SegmentStream, make_vad

FRAME_S = 0.08   # openWakeWord wants 80 ms frames at 16 kHz


class EarNode(Node):
    def __init__(self):
        super().__init__('voice_ear')

        # Device is a PortAudio name substring (e.g. 'Camera'), resolved at
        # capture time — ALSA card indices shift across boots, names don't.
        self.declare_parameter('capture_device', 'Camera')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('listen_window_s', 6.0)
        self.declare_parameter('silence_after_speech_s', 0.8)
        self.declare_parameter('min_speech_s', 0.2)
        self.declare_parameter('pre_roll_s', 0.3)
        self.declare_parameter('vad_backend', 'auto')      # auto | silero | energy
        self.declare_parameter('vad_threshold', 0.5)       # silero speech prob
        self.declare_parameter('energy_threshold', 0.015)  # RMS, 0..1 (energy gate)
        self.declare_parameter('chime_lead_s', 0.3)        # mouth's chime plays in this gap
        self.declare_parameter('asr_model', 'base.en')
        self.declare_parameter('models_dir', '/home/ws/ugv_ws/models')
        self.declare_parameter('wake_word_enabled', False)
        self.declare_parameter('wake_word_model', 'hey_jarvis')
        self.declare_parameter('wake_word_threshold', 0.6)
        self.declare_parameter('record_max_s', 15.0)
        self.declare_parameter('stop_watch_during_motion', True)
        self.declare_parameter('stop_watch_while_speaking', False)  # needs AEC mic
        self.declare_parameter('stop_watch_max_segment_s', 2.0)

        self.sample_rate = int(self.get_parameter('sample_rate').value)
        if self.sample_rate != SILERO_RATE:
            self.get_logger().warn('sample_rate %d != %d — VAD/wake models expect %d'
                                   % (self.sample_rate, SILERO_RATE, SILERO_RATE))
        self.listen_window_s = float(self.get_parameter('listen_window_s').value)
        self.silence_s = float(self.get_parameter('silence_after_speech_s').value)
        self.min_speech_s = float(self.get_parameter('min_speech_s').value)
        self.pre_roll_s = float(self.get_parameter('pre_roll_s').value)
        self.chime_lead_s = float(self.get_parameter('chime_lead_s').value)
        self.wake_enabled = bool(self.get_parameter('wake_word_enabled').value)
        self.stop_watch_during_motion = bool(
            self.get_parameter('stop_watch_during_motion').value)
        self.stop_watch_while_speaking = bool(
            self.get_parameter('stop_watch_while_speaking').value)

        self.transcript_pub = self.create_publisher(Transcript, '/voice/transcript', 10)
        self.estop_pub = self.create_publisher(Empty, '/voice/estop', 10)
        self.listening_pub = self.create_publisher(Bool, '/voice/listening', 10)
        self.estop_client = self.create_client(Trigger, 'behavior/estop')

        self.create_subscription(Empty, '/voice/listen_once', self.listen_once_callback, 10)
        self.create_subscription(Bool, '/voice/speaking', self.speaking_callback, 10)
        self.create_subscription(
            Bool, 'behavior/motion_active', self.motion_active_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Record, 'voice/record', self.record_callback)

        self._speaking = False
        self._motion_active = False
        self._listen_requested = threading.Event()
        self._record_requested = threading.Event()   # (duration, path) pending
        self._record_done = threading.Event()
        self._record_job = None
        self._record_result = None
        self._asr = None
        self._asr_lock = threading.Lock()
        self._asr_label = 'none'
        self._vad = None
        self._wake_model = None
        self._capture_rate = None  # resolved on first successful device open
        self._stop_queue = queue.Queue(maxsize=4)
        self._stop_hits = 0

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self._stop_thread = threading.Thread(target=self._stop_watch_loop, daemon=True)
        self._stop_thread.start()

        mode = 'wake word' if self.wake_enabled else 'push-to-talk (/voice/listen_once)'
        self.get_logger().info(
            'voice_ear up — mode: %s; stop watch during motion: %s, while speaking: %s'
            % (mode, self.stop_watch_during_motion, self.stop_watch_while_speaking))

    # --- callbacks --------------------------------------------------------

    def speaking_callback(self, msg):
        self._speaking = bool(msg.data)

    def motion_active_callback(self, msg):
        self._motion_active = bool(msg.data)

    def listen_once_callback(self, _msg):
        if self._speaking:
            self.get_logger().info('listen_once ignored: robot is speaking')
            return
        if self._listen_requested.is_set():
            self.get_logger().info('listen_once ignored: already listening')
            return
        self._listen_requested.set()

    def record_callback(self, request, response):
        """Hand a fixed-length capture to the worker thread and wait for it.
        Blocking the executor here is deliberate: nothing else the ear does
        matters while the mic is recording, and the call is bounded."""
        max_s = float(self.get_parameter('record_max_s').value)
        duration = min(max_s, max(0.5, float(request.duration_s)))
        if self._record_requested.is_set() or self._listen_requested.is_set():
            response.success = False
            response.message = 'ear busy'
            return response
        self._record_job = (duration, request.path)
        self._record_result = None
        self._record_done.clear()
        self._record_requested.set()
        if not self._record_done.wait(timeout=duration + 10.0):
            self._record_requested.clear()
            response.success = False
            response.message = 'record timed out'
            return response
        ok, path, message = self._record_result
        response.success = ok
        response.path = path
        response.message = message
        return response

    # --- worker -----------------------------------------------------------

    def _worker_loop(self):
        # Warm-load the models so the first utterance doesn't pay for them.
        try:
            self._load_asr()
        except Exception as e:
            self.get_logger().warn(
                f'ASR warm-load failed (will retry on first use): {e}')
        self._vad = make_vad(str(self.get_parameter('vad_backend').value),
                             threshold=float(self.get_parameter('vad_threshold').value),
                             energy_threshold=float(self.get_parameter('energy_threshold').value),
                             logger=self.get_logger())
        self.get_logger().info('VAD backend: %s' % self._vad.name)
        while rclpy.ok():
            # Everything in the cycle is guarded: an exception (bad device,
            # broken wake stack, model load failure) must never kill this
            # thread — the ear just logs and keeps listening.
            try:
                if self._listen_requested.is_set():
                    try:
                        audio = self._record_utterance()
                        if audio is not None:
                            self._transcribe_and_publish(audio)
                    finally:
                        self._listen_requested.clear()
                elif self._record_requested.is_set():
                    self._run_record_job()
                elif self.wake_enabled or self._stop_watch_wanted():
                    self._monitor()
                else:
                    self._listen_requested.wait(timeout=0.2)
            except Exception as e:
                self.get_logger().error(f'listen cycle failed: {e}')
                self._listen_requested.clear()
                time.sleep(0.5)

    def _stop_watch_wanted(self):
        return ((self.stop_watch_during_motion and self._motion_active)
                or (self.stop_watch_while_speaking and self._speaking))

    # --- listening (one utterance) ----------------------------------------

    def _record_utterance(self):
        """Record one utterance: VAD-endpointed, capped at the listen
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
        segmenter = SegmentStream(self._vad, min_speech_s=self.min_speech_s,
                                  silence_after_speech_s=self.silence_s,
                                  max_utterance_s=self.listen_window_s,
                                  pre_roll_s=self.pre_roll_s)
        segmenter.reset()
        self._publish_listening(True)
        if self.chime_lead_s > 0:
            time.sleep(self.chime_lead_s)   # the mouth's chime plays here
        self.get_logger().info('listening…')
        utterance = None
        try:
            stream, capture_rate = self._open_input_stream(sd, device, 'float32', FRAME_S)
            block = int(capture_rate * FRAME_S)
            deadline = time.monotonic() + self.listen_window_s + 1.0
            with stream:
                while time.monotonic() < deadline:
                    data, _overflow = stream.read(block)
                    frame = self._resample(np, data[:, 0], capture_rate, self.sample_rate)
                    done = False
                    for ev, seg in segmenter.feed(frame):
                        if ev in ('end', 'timeout'):
                            utterance = seg
                            done = True
                    if done:
                        break
        except Exception as e:
            self._capture_rate = None  # device may have changed — re-probe
            self.get_logger().error(
                f'mic capture failed on device {device!r}: {e} — devices: '
                f'{[d["name"] for d in sd.query_devices()]}')
            return None
        finally:
            self._publish_listening(False)

        if utterance is None:
            self._publish_transcript('', 0.0)  # "heard nothing"
            return None
        return utterance.astype(np.float32)

    # --- fixed-length recording (record_replay) ---------------------------

    def _run_record_job(self):
        duration, path = self._record_job
        try:
            self._record_result = self._record_fixed(duration, path)
        except Exception as e:
            self._capture_rate = None
            self._record_result = (False, '', 'record failed: %s' % e)
            self.get_logger().error('record failed: %s' % e)
        finally:
            self._record_requested.clear()
            self._record_done.set()

    def _record_fixed(self, duration, path):
        """Capture exactly `duration` seconds to a 16-bit mono wav at the
        device's capture rate (no resampling — playback keeps the rate)."""
        import numpy as np
        import sounddevice as sd
        from .audio_fx import write_wav_mono16

        device = self._resolve_device(sd)
        stream, capture_rate = self._open_input_stream(sd, device, 'int16', 0.05)
        block = int(capture_rate * 0.05)
        total = int(capture_rate * duration)
        chunks, got = [], 0
        self.get_logger().info('recording %.1f s to %s' % (duration, path))
        with stream:
            while got < total:
                data, _overflow = stream.read(min(block, total - got))
                mono = np.ascontiguousarray(data[:, 0])
                chunks.append(mono.copy())
                got += len(mono)
        pcm = np.concatenate(chunks).astype(np.int16)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        write_wav_mono16(path, pcm.tobytes(), capture_rate)
        self.get_logger().info('recorded %d samples at %d Hz' % (len(pcm), capture_rate))
        return (True, path, 'ok')

    # --- monitor: wake word + continuous stop watch -----------------------

    def _monitor(self):
        """Keep the mic open and run the wake-word model (when not
        speaking) and the stop watch (while moving / speaking, per params)
        on every frame. Returns when a listen/record request arrives, on
        a wake hit (which sets the listen flag), or when neither detector
        is wanted any more."""
        import numpy as np
        import sounddevice as sd

        wake = self._load_wake_model() if self.wake_enabled else None
        threshold = float(self.get_parameter('wake_word_threshold').value)
        device = self._resolve_device(sd)
        segmenter = SegmentStream(
            self._vad, min_speech_s=0.15, silence_after_speech_s=0.25,
            max_utterance_s=float(self.get_parameter('stop_watch_max_segment_s').value),
            pre_roll_s=0.2)
        stream, capture_rate = self._open_input_stream(sd, device, 'int16', FRAME_S)
        block = int(capture_rate * FRAME_S)
        watching = False
        with stream:
            while rclpy.ok():
                if self._listen_requested.is_set() or self._record_requested.is_set():
                    return
                want_stop = self._stop_watch_wanted()
                if not self.wake_enabled and not want_stop:
                    return
                data, _ = stream.read(block)
                frame = self._resample(np, np.squeeze(data), capture_rate, self.sample_rate)
                speaking = self._speaking

                if wake is not None and not speaking:   # half-duplex mute
                    scores = wake.predict(frame)
                    if any(s >= threshold for s in scores.values()):
                        self.get_logger().info('wake word detected')
                        wake.reset()
                        self._listen_requested.set()
                        return

                if want_stop:
                    if not watching:
                        segmenter.reset()
                        watching = True
                        self.get_logger().info('stop watch on')
                    for ev, seg in segmenter.feed(frame.astype(np.float32) / 32768.0):
                        if ev in ('end', 'timeout'):
                            try:
                                self._stop_queue.put_nowait(seg)
                            except queue.Full:
                                self.get_logger().warn('stop watch: ASR backlog, segment dropped')
                elif watching:
                    watching = False
                    self.get_logger().info('stop watch off')

    def _stop_watch_loop(self):
        """Transcribe stop-watch segments off the mic thread; a stop-lexicon
        hit fires the estop path exactly like a normal transcript would."""
        while rclpy.ok():
            try:
                seg = self._stop_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                asr = self._load_asr()
                if asr is None:
                    continue
                t0 = time.monotonic()
                with self._asr_lock:
                    segments, _ = asr.transcribe(seg, language='en', beam_size=1,
                                                 vad_filter=False,
                                                 condition_on_previous_text=False)
                    text = ' '.join(s.text.strip() for s in segments).strip()
                dt = time.monotonic() - t0
                if text and contains_stop_word(text):
                    self._stop_hits += 1
                    self.get_logger().warn(
                        'stop watch: STOP heard (%r, ASR %.2f s)' % (text, dt))
                    self._publish_transcript(text, 0.0)   # fires estop, stop_word=True
                elif text:
                    self.get_logger().debug('stop watch heard %r (%.2f s)' % (text, dt))
            except Exception as e:
                self.get_logger().error('stop watch ASR failed: %s' % e)

    # --- devices ----------------------------------------------------------

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
        often 44.1/48 kHz only), reopen at the device's native rate. The
        working rate is cached so later cycles skip the failed open. Returns
        (unstarted stream, actual capture rate); callers resample to
        sample_rate."""
        if self._capture_rate is not None:
            block = int(self._capture_rate * block_s)
            return (sd.InputStream(device=device, channels=1, dtype=dtype,
                                   samplerate=self._capture_rate,
                                   blocksize=block),
                    self._capture_rate)
        try:
            block = int(self.sample_rate * block_s)
            stream = sd.InputStream(device=device, channels=1, dtype=dtype,
                                    samplerate=self.sample_rate,
                                    blocksize=block)
            self._capture_rate = self.sample_rate
            return stream, self.sample_rate
        except sd.PortAudioError:
            native = int(sd.query_devices(device, kind='input')
                         ['default_samplerate'])
            self.get_logger().warn(
                f'device {device!r} cannot capture at {self.sample_rate} Hz '
                f'— capturing at {native} Hz and resampling')
            block = int(native * block_s)
            stream = sd.InputStream(device=device, channels=1, dtype=dtype,
                                    samplerate=native, blocksize=block)
            self._capture_rate = native
            return stream, native

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
        with self._asr_lock:
            if self._asr is None:
                self._asr = WhisperModel(model_ref, device='cpu', compute_type='int8',
                                         download_root=models_dir)
                self._asr_label = f'faster-whisper-{model_name}-int8'
                self.get_logger().info(f'loaded ASR: {self._asr_label}')
        return self._asr

    def _transcribe_and_publish(self, audio):
        asr = self._load_asr()
        if asr is None:
            return
        with self._asr_lock:
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

    def _publish_listening(self, value):
        self.listening_pub.publish(Bool(data=bool(value)))

    def _fire_estop(self):
        """Stop path independent of the brain: service call + observable topic."""
        self.get_logger().warn('stop word heard — calling behavior/estop')
        self.estop_pub.publish(Empty())
        if self.estop_client.service_is_ready():
            self.estop_client.call_async(Trigger.Request())
        else:
            self.get_logger().error('behavior/estop service not available!')

    # --- wake word --------------------------------------------------------

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

    def _load_wake_model(self):
        if self._wake_model is not None:
            return self._wake_model
        try:
            from openwakeword.model import Model as WakeModel
        except ImportError as e:
            self.get_logger().error(
                f'wake word unavailable ({e}) — falling back to push-to-talk')
            self.wake_enabled = False
            return None
        self._wake_model = WakeModel(
            wakeword_models=[self._resolve_wake_model()],
            inference_framework='onnx',
            **self._oww_feature_paths())
        self.get_logger().info('wake model loaded: %s' % self._resolve_wake_model())
        return self._wake_model


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
