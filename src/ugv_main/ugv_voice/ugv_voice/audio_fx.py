"""Pure-Python audio helpers for the mouth's wav playback (record_replay).

Tape-deck speed/pitch shift, exactly yak_back.sh's trick: replaying the
same samples at a scaled rate makes speech faster *and* higher (1.3 =
chipmunk) or slower *and* deeper (0.7). Here the samples are resampled
instead so the output rate stays whatever the recording was — aplay never
sees an odd rate the hardware might refuse. numpy only, no ROS.
"""

import wave

SPEED_MIN = 0.5
SPEED_MAX = 2.0


def clamp_speed(speed):
    try:
        s = float(speed)
    except (TypeError, ValueError):
        return 1.0
    if s != s:  # NaN
        return 1.0
    return min(SPEED_MAX, max(SPEED_MIN, s))


def read_wav_mono16(path):
    """Load a wav as (int16 PCM bytes, rate). Stereo -> first channel;
    8/32-bit is converted to 16-bit so aplay gets one fixed format."""
    import numpy as np
    with wave.open(path, 'rb') as w:
        channels, width, rate = w.getnchannels(), w.getsampwidth(), w.getframerate()
        frames = w.readframes(w.getnframes())
    if width == 2:
        data = np.frombuffer(frames, dtype=np.int16)
    elif width == 4:
        data = (np.frombuffer(frames, dtype=np.int32) >> 16).astype(np.int16)
    elif width == 1:
        data = ((np.frombuffer(frames, dtype=np.uint8).astype(np.int16) - 128) << 8)
    else:
        raise ValueError('unsupported wav sample width: %d' % width)
    if channels > 1:
        data = data[::channels]
    return data.tobytes(), rate


def tape_deck(pcm_int16, speed):
    """Resample int16 PCM so it plays `speed` times faster (and higher).
    speed is clamped to [SPEED_MIN, SPEED_MAX]; 1.0 returns the input."""
    import numpy as np
    speed = clamp_speed(speed)
    if speed == 1.0:
        return pcm_int16
    src = np.frombuffer(pcm_int16, dtype=np.int16).astype(np.float32)
    if len(src) < 2:
        return pcm_int16
    n = max(1, int(round(len(src) / speed)))
    positions = np.linspace(0.0, len(src) - 1, n)
    out = np.interp(positions, np.arange(len(src)), src)
    return np.clip(np.rint(out), -32768, 32767).astype(np.int16).tobytes()


def write_wav_mono16(path, pcm_int16, rate):
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(rate))
        w.writeframes(pcm_int16)
