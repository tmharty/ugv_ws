"""Tape-deck resampling + wav IO for the record_replay playback."""

import os
import tempfile

import numpy as np
import pytest

from ugv_voice.audio_fx import (SPEED_MAX, SPEED_MIN, clamp_speed,
                                read_wav_mono16, tape_deck, write_wav_mono16)


def tone(seconds=0.5, rate=16000, hz=440.0):
    t = np.arange(int(seconds * rate)) / rate
    return (np.sin(2 * np.pi * hz * t) * 12000).astype(np.int16).tobytes()


def test_clamp_speed():
    assert clamp_speed(1.3) == 1.3
    assert clamp_speed(99) == SPEED_MAX
    assert clamp_speed(0.01) == SPEED_MIN
    assert clamp_speed('nope') == 1.0
    assert clamp_speed(float('nan')) == 1.0
    assert clamp_speed(None) == 1.0


def test_tape_deck_identity_at_1():
    pcm = tone()
    assert tape_deck(pcm, 1.0) is pcm


def test_tape_deck_shortens_for_chipmunk_and_lengthens_for_deep():
    pcm = tone()
    n = len(pcm) // 2
    fast = tape_deck(pcm, 1.3)
    slow = tape_deck(pcm, 0.7)
    assert abs(len(fast) // 2 - n / 1.3) <= 1
    assert abs(len(slow) // 2 - n / 0.7) <= 1
    assert len(fast) % 2 == 0 and len(slow) % 2 == 0


def test_tape_deck_shifts_pitch():
    """Dominant frequency scales with speed — the tape-deck effect."""
    rate = 16000
    pcm = tone(1.0, rate, 440.0)

    def peak_hz(b):
        x = np.frombuffer(b, dtype=np.int16).astype(np.float64)
        f = np.fft.rfftfreq(len(x), 1.0 / rate)
        return f[np.argmax(np.abs(np.fft.rfft(x)))]
    assert abs(peak_hz(pcm) - 440) < 5
    assert abs(peak_hz(tape_deck(pcm, 1.3)) - 440 * 1.3) < 8
    assert abs(peak_hz(tape_deck(pcm, 0.7)) - 440 * 0.7) < 8


def test_tape_deck_clamps_and_survives_tiny_input():
    pcm = tone()
    assert len(tape_deck(pcm, 100.0)) == len(tape_deck(pcm, SPEED_MAX))
    assert tape_deck(b'\x00\x00', 1.5) == b'\x00\x00'
    assert tape_deck(b'', 1.5) == b''


def test_wav_roundtrip_and_stereo_downmix(tmp_path):
    pcm = tone(0.2, 8000)
    path = str(tmp_path / 'a.wav')
    write_wav_mono16(path, pcm, 8000)
    got, rate = read_wav_mono16(path)
    assert rate == 8000 and got == pcm

    import wave
    stereo = str(tmp_path / 's.wav')
    left = np.frombuffer(pcm, dtype=np.int16)
    inter = np.empty(len(left) * 2, dtype=np.int16)
    inter[0::2] = left
    inter[1::2] = 0
    with wave.open(stereo, 'wb') as w:
        w.setnchannels(2); w.setsampwidth(2); w.setframerate(8000)
        w.writeframes(inter.tobytes())
    got, rate = read_wav_mono16(stereo)
    assert got == pcm


def test_unsupported_width_raises(tmp_path):
    import wave
    path = str(tmp_path / 'w3.wav')
    with wave.open(path, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(3); w.setframerate(8000)
        w.writeframes(b'\x00' * 30)
    with pytest.raises(ValueError):
        read_wav_mono16(path)
