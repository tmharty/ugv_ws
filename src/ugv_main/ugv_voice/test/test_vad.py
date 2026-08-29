"""Endpointer state machine, energy VAD, SegmentStream, chime. Silero
itself is exercised when faster-whisper is importable (the container)."""

import numpy as np
import pytest

from ugv_voice.audio_fx import chime
from ugv_voice.vad import (SILERO_CHUNK, SILERO_RATE, EnergyVAD, Endpointer,
                           SegmentStream, make_vad)

CH = SILERO_CHUNK / SILERO_RATE   # 32 ms


def run(ep, flags):
    return [ep.feed(i, f) for i, f in enumerate(flags)]


def test_endpointer_needs_min_speech_before_start():
    ep = Endpointer(CH, min_speech_s=3 * CH, silence_after_speech_s=2 * CH)
    assert run(ep, [1, 0, 1, 1]) == [None, None, None, None]   # runs reset on silence
    assert ep.feed(4, 1) == 'start'


def test_endpointer_end_after_trailing_silence_and_pre_roll_kept():
    ep = Endpointer(CH, min_speech_s=2 * CH, silence_after_speech_s=3 * CH,
                    pre_roll_s=4 * CH)
    evs = run(ep, [0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0])
    assert evs[6] == 'start'
    assert evs[-1] == 'end'
    chunks = ep.take()
    # pre-roll: the 4 chunks before start (2 silence + the 2 speech that
    # triggered it), then everything after
    assert chunks == [3, 4, 5, 6, 7, 8, 9, 10]
    assert not ep.started


def test_endpointer_timeout_caps_utterance():
    ep = Endpointer(CH, min_speech_s=CH, silence_after_speech_s=10 * CH,
                    max_utterance_s=5 * CH, pre_roll_s=CH)
    evs = run(ep, [1] * 8)
    assert evs[0] == 'start'
    assert 'timeout' in evs
    assert evs.index('timeout') == 4      # 5 chunks collected (pre-roll incl.)


def test_endpointer_speech_gap_shorter_than_silence_does_not_end():
    ep = Endpointer(CH, min_speech_s=CH, silence_after_speech_s=3 * CH)
    evs = run(ep, [1, 0, 0, 1, 0, 0, 1])
    assert evs == ['start', None, None, None, None, None, None]


def test_energy_vad_probs_and_buffering():
    v = EnergyVAD(threshold=0.1)
    loud = np.full(SILERO_CHUNK, 0.5, dtype=np.float32)
    quiet = np.zeros(SILERO_CHUNK, dtype=np.float32)
    assert v.probs(np.concatenate([loud, quiet])) == [1.0, 0.0]
    assert v.probs(loud[:100]) == []                 # partial chunk waits
    assert v.probs(loud[100:]) == [1.0]              # ...and completes


def test_segment_stream_yields_utterance_audio():
    vad = EnergyVAD(threshold=0.1)
    ss = SegmentStream(vad, min_speech_s=2 * CH, silence_after_speech_s=3 * CH,
                       max_utterance_s=2.0, pre_roll_s=2 * CH)
    n = SILERO_CHUNK
    audio = np.concatenate([np.zeros(6 * n), np.full(10 * n, 0.5), np.zeros(6 * n)]).astype(np.float32)
    events = []
    for i in range(0, len(audio), 1280):            # 80 ms frames like the ear
        events.extend(ss.feed(audio[i:i + 1280]))
    kinds = [e for e, _ in events]
    assert kinds == ['start', 'end']
    seg = events[1][1]
    # 2 pre-roll + 8 more speech + 3 silence chunks
    assert len(seg) == (2 + 8 + 3) * n
    assert seg.max() == 0.5


def test_make_vad_falls_back_to_energy():
    v = make_vad('nonsense')
    assert v.name == 'energy'


def test_chime_is_short_and_bounded():
    pcm = chime(16000, volume=0.3)
    x = np.frombuffer(pcm, dtype=np.int16)
    assert 0.15 < len(x) / 16000 < 0.25
    assert np.abs(x).max() <= 0.3 * 32767 + 1
    assert x[0] == 0 and abs(x[-1]) < 500       # enveloped edges, no click


# --- Silero (container only) --------------------------------------------

def _need_silero():
    pytest.importorskip('faster_whisper.vad', reason='faster-whisper not installed')


def _espeak_16k(tmp_path, text):
    import shutil
    import subprocess
    import wave
    if not shutil.which('espeak-ng'):
        pytest.skip('espeak-ng missing')
    wav = str(tmp_path / 'tts.wav')
    subprocess.run(['espeak-ng', '-w', wav, text], check=True, capture_output=True)
    with wave.open(wav, 'rb') as w:
        rate = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
    audio = pcm.astype(np.float32) / 32768.0
    if rate != SILERO_RATE:
        n = int(len(audio) * SILERO_RATE / rate)
        audio = np.interp(np.linspace(0, len(audio) - 1, n), np.arange(len(audio)), audio).astype(np.float32)
    return np.concatenate([np.zeros(SILERO_RATE // 2, np.float32), audio,
                           np.zeros(SILERO_RATE, np.float32)])


def test_silero_stream_silence_vs_speechlike():
    _need_silero()
    from ugv_voice.vad import SileroStreamVAD
    v = SileroStreamVAD(threshold=0.5)
    silence = np.zeros(SILERO_RATE, dtype=np.float32)
    ps = v.probs(silence)
    assert len(ps) == SILERO_RATE // SILERO_CHUNK
    assert max(ps) < 0.2
    # Streaming must match batch: feeding in odd-sized pieces gives the
    # same count of probs as one call.
    v.reset()
    pieces = [silence[:700], silence[700:2000], silence[2000:]]
    assert sum(len(v.probs(p)) for p in pieces) == SILERO_RATE // SILERO_CHUNK


def test_silero_detects_synthesized_speech(tmp_path):
    """espeak-ng renders 'stop' -> Silero must flag it as speech."""
    _need_silero()
    from ugv_voice.vad import SileroStreamVAD
    padded = _espeak_16k(tmp_path, 'stop')
    v = SileroStreamVAD()
    ps = v.probs(padded)
    assert max(ps) > 0.5
    ss = SegmentStream(SileroStreamVAD(), min_speech_s=0.1, silence_after_speech_s=0.3,
                       max_utterance_s=2.0, pre_roll_s=0.2)
    events = []
    for i in range(0, len(padded), 1280):
        events.extend(ss.feed(padded[i:i + 1280]))
    assert [e for e, _ in events][:2] == ['start', 'end']


def test_stop_watch_chain_espeak_to_estop_decision(tmp_path):
    """The whole continuous-stop path without a mic: synthesized 'robot
    stop' -> Silero segmentation (as the monitor does, 80 ms frames) ->
    faster-whisper on the segment -> stop lexicon. Also checks a
    non-stop phrase does NOT trigger."""
    import os
    _need_silero()
    from faster_whisper import WhisperModel
    from ugv_voice.intent_schema import contains_stop_word
    from ugv_voice.vad import SileroStreamVAD
    model_dir = '/home/ws/ugv_ws/models/faster-whisper-base.en'
    if not os.path.isdir(model_dir):
        pytest.skip('whisper model not fetched (scripts/fetch_voice_models.sh)')
    asr = WhisperModel(model_dir, device='cpu', compute_type='int8')

    def heard(text):
        ss = SegmentStream(SileroStreamVAD(), min_speech_s=0.15, silence_after_speech_s=0.25,
                           max_utterance_s=2.0, pre_roll_s=0.2)
        audio = _espeak_16k(tmp_path, text)
        segs = []
        for i in range(0, len(audio), 1280):
            segs.extend(seg for ev, seg in ss.feed(audio[i:i + 1280]) if ev in ('end', 'timeout'))
        assert segs, 'no segment for %r' % text
        out = []
        for seg in segs:
            segments, _ = asr.transcribe(seg, language='en', beam_size=1, vad_filter=False)
            out.append(' '.join(s.text.strip() for s in segments))
        return ' '.join(out)

    assert contains_stop_word(heard('robot stop'))
    assert not contains_stop_word(heard('hello robot'))
