"""Voice activity detection + endpointing for the ear. No ROS imports.

Three pieces:

  * ``SileroStreamVAD`` — streaming wrapper over the Silero VAD v6 ONNX
    model that faster-whisper ships (no extra download). Keeps the
    recurrent state between calls so it works frame by frame; buffers
    arbitrary input sizes into the 512-sample (32 ms @ 16 kHz) chunks the
    model wants.
  * ``EnergyVAD`` — RMS threshold with the same ``probs()`` interface; the
    fallback when onnxruntime/faster-whisper are missing.
  * ``Endpointer`` — pure state machine turning per-chunk speech flags into
    start / end / timeout events and collecting the utterance audio
    (with pre-roll so the first syllable is not clipped).

``SegmentStream`` glues a VAD and an Endpointer together for the ear's
always-on monitor: feed 16 kHz float audio, get finished utterances back.
"""

import collections

SILERO_RATE = 16000
SILERO_CHUNK = 512
SILERO_CONTEXT = 64


class SileroStreamVAD:
    """Streaming Silero VAD. probs(audio) -> speech probability per
    complete 512-sample chunk (leftover samples wait for the next call)."""

    name = 'silero'

    def __init__(self, threshold=0.5, model_path=None):
        import numpy as np
        self._np = np
        self.threshold = float(threshold)
        if model_path is None:
            import os
            from faster_whisper.vad import get_assets_path
            model_path = os.path.join(get_assets_path(), 'silero_vad_v6.onnx')
        from faster_whisper.vad import SileroVADModel
        self._session = SileroVADModel(model_path).session
        self._pending = np.zeros(0, dtype=np.float32)
        self.reset()

    def reset(self):
        np = self._np
        self._h = np.zeros((1, 1, 128), dtype=np.float32)
        self._c = np.zeros((1, 1, 128), dtype=np.float32)
        self._context = np.zeros(SILERO_CONTEXT, dtype=np.float32)
        self._pending = np.zeros(0, dtype=np.float32)

    def probs(self, audio):
        """audio: float32 mono at 16 kHz, any length, -1..1."""
        np = self._np
        buf = np.concatenate([self._pending, np.asarray(audio, dtype=np.float32)])
        out = []
        n_full = len(buf) // SILERO_CHUNK
        for i in range(n_full):
            chunk = buf[i * SILERO_CHUNK:(i + 1) * SILERO_CHUNK]
            x = np.concatenate([self._context, chunk])[None, :]
            o, self._h, self._c = self._session.run(
                None, {'input': x, 'h': self._h, 'c': self._c})
            out.append(float(np.asarray(o).reshape(-1)[0]))
            self._context = chunk[-SILERO_CONTEXT:]
        self._pending = buf[n_full * SILERO_CHUNK:]
        return out

    def is_speech(self, prob):
        return prob >= self.threshold


class EnergyVAD:
    """RMS gate with the same interface. threshold on the 0..1 scale."""

    name = 'energy'

    def __init__(self, threshold=0.015):
        import numpy as np
        self._np = np
        self.threshold = float(threshold)
        self._pending = np.zeros(0, dtype=np.float32)

    def reset(self):
        self._pending = self._np.zeros(0, dtype=self._np.float32)

    def probs(self, audio):
        np = self._np
        buf = np.concatenate([self._pending, np.asarray(audio, dtype=np.float32)])
        out = []
        n_full = len(buf) // SILERO_CHUNK
        for i in range(n_full):
            chunk = buf[i * SILERO_CHUNK:(i + 1) * SILERO_CHUNK]
            rms = float(np.sqrt(np.mean(chunk ** 2)))
            out.append(1.0 if rms >= self.threshold else 0.0)
        self._pending = buf[n_full * SILERO_CHUNK:]
        return out

    def is_speech(self, prob):
        return prob >= 0.5


def make_vad(backend='auto', threshold=0.5, energy_threshold=0.015, logger=None):
    """'auto' -> Silero if its deps import, else energy. Never raises."""
    if backend in ('auto', 'silero'):
        try:
            return SileroStreamVAD(threshold=threshold)
        except Exception as e:
            if backend == 'silero' or logger is not None:
                (logger.warn if logger else print)(
                    'silero VAD unavailable (%s) — using energy gate' % e)
    return EnergyVAD(threshold=energy_threshold)


class Endpointer:
    """Per-chunk speech flags -> utterance events, collecting the audio.

    feed(chunk, is_speech) returns None, 'start', 'end' or 'timeout'.
    After 'end'/'timeout', take() returns the utterance (pre-roll +
    speech + trailing silence) and resets for the next one.
    """

    def __init__(self, chunk_s, min_speech_s=0.2, silence_after_speech_s=0.6,
                 max_utterance_s=6.0, pre_roll_s=0.3):
        self.chunk_s = float(chunk_s)
        self.min_speech_chunks = max(1, int(round(min_speech_s / chunk_s)))
        self.silence_chunks = max(1, int(round(silence_after_speech_s / chunk_s)))
        self.max_chunks = max(1, int(round(max_utterance_s / chunk_s)))
        self._pre_roll = collections.deque(
            maxlen=max(1, int(round(pre_roll_s / chunk_s))))
        self.reset()

    def reset(self):
        self.started = False
        self._speech_run = 0
        self._quiet_run = 0
        self._chunks = []
        self._pre_roll.clear()

    def feed(self, chunk, is_speech):
        if not self.started:
            self._pre_roll.append(chunk)
            self._speech_run = self._speech_run + 1 if is_speech else 0
            if self._speech_run >= self.min_speech_chunks:
                self.started = True
                self._chunks = list(self._pre_roll)
                self._pre_roll.clear()
                self._quiet_run = 0
                return 'start'
            return None
        self._chunks.append(chunk)
        if is_speech:
            self._quiet_run = 0
        else:
            self._quiet_run += 1
            if self._quiet_run >= self.silence_chunks:
                return 'end'
        if len(self._chunks) >= self.max_chunks:
            return 'timeout'
        return None

    def take(self):
        chunks = self._chunks
        self.reset()
        return chunks

    @property
    def speech_s(self):
        return len(self._chunks) * self.chunk_s


class SegmentStream:
    """Feed 16 kHz float audio; get finished utterances (numpy arrays)."""

    def __init__(self, vad, **endpointer_kwargs):
        self.vad = vad
        self.endpointer = Endpointer(SILERO_CHUNK / SILERO_RATE, **endpointer_kwargs)
        self._pending = None

    def reset(self):
        self.vad.reset()
        self.endpointer.reset()
        self._pending = None

    def feed(self, audio):
        """Returns a list of (event, samples) for events that fired."""
        import numpy as np
        audio = np.asarray(audio, dtype=np.float32)
        probs = self.vad.probs(audio)
        if not probs:
            return []
        # Chunks the VAD consumed this call: reconstruct them from the
        # trailing pending buffer the VAD keeps (it holds < 1 chunk).
        consumed = len(probs) * SILERO_CHUNK
        carried = self._pending if self._pending is not None else np.zeros(0, np.float32)
        buf = np.concatenate([carried, audio])
        chunks = [buf[i * SILERO_CHUNK:(i + 1) * SILERO_CHUNK] for i in range(len(probs))]
        self._pending = buf[consumed:]
        events = []
        for chunk, p in zip(chunks, probs):
            ev = self.endpointer.feed(chunk, self.vad.is_speech(p))
            if ev in ('end', 'timeout'):
                events.append((ev, np.concatenate(self.endpointer.take())))
            elif ev == 'start':
                events.append(('start', None))
        return events
