"""Deterministic validator between any Brain and the robot.

Every brain backend (rules, ollama, bedrock) emits a raw intent dict:

    {"intent": "move_forward", "params": {"distance_m": 0.3}}

This module is the only path from that dict to a Behavior action goal. It
enforces an intent allow-list, per-parameter clamps, and strict input shape
(unknown keys, compound commands, and non-finite numbers are rejected), then
renders the exact Behavior JSON string `behavior_ctrl` will parse.

Pure Python by design: zero ROS imports, injectable clock, so the whole safety
surface is unit-testable in plain pytest with no ROS runtime.
"""

import json
import math
import re
from dataclasses import dataclass, field
from typing import Optional


# --- stop lexicon (shared with voice_ear's scanner) -----------------------

# Phrases first (matched as-is), then single words with boundaries. False
# positives are safe here — they only ever stop the robot.
_STOP_PHRASES = (
    "emergency stop",
    "stand still",
    "stay still",
    "don't move",
    "dont move",
    "no more",
)
_STOP_WORDS = ("stop", "halt", "freeze", "whoa", "woah")

_STOP_RE = re.compile(
    r"\b(" + "|".join(_STOP_PHRASES + _STOP_WORDS) + r")\b")


def contains_stop_word(text):
    """True if the utterance contains anything in the stop lexicon."""
    return bool(_STOP_RE.search(text.lower()))


# --- intent registry ------------------------------------------------------

# Numeric-parameter motion intents: param name, default, lo, hi.
# Ranges are the plan's kid-safe envelope; behavior_ctrl clamps again below us.
NUMERIC_INTENTS = {
    'move_forward':  ('distance_m', 0.3, 0.1, 1.0),
    'move_backward': ('distance_m', 0.2, 0.1, 0.5),
    'turn_left':     ('angle_deg', 90.0, 15.0, 180.0),
    'turn_right':    ('angle_deg', 90.0, 15.0, 180.0),
}

POINT_NAMES = ('a', 'b', 'c', 'd', 'e', 'f', 'g')
POINT_INTENTS = ('go_to_point', 'save_point')

# Speech-only: the brain node answers from the response library; no motion.
SPEECH_INTENTS = ('greeting', 'who_are_you', 'what_can_you_do',
                  'battery_status', 'affirm', 'deny', 'unknown')

DEVICE_INTENTS = ('led_on', 'led_off', 'led_blink')

FIXED_INTENTS = ('spin_around', 'stop')

ALL_INTENTS = frozenset(
    tuple(NUMERIC_INTENTS) + POINT_INTENTS + SPEECH_INTENTS
    + DEVICE_INTENTS + FIXED_INTENTS)


@dataclass(frozen=True)
class ValidatedIntent:
    """The only object the brain node acts on."""
    name: str
    params: dict = field(default_factory=dict)
    behavior_json: Optional[str] = None   # None => nothing goes to behavior_ctrl
    reply_key: str = 'unknown'
    clamp_notes: tuple = ()               # human-readable, for logging
    requires_confirmation: bool = False   # go_to_point: wait for verbal "yes"
    is_stop: bool = False
    rejected_reason: Optional[str] = None  # non-None => resolved to a refusal

    @property
    def is_motion(self):
        return self.behavior_json is not None and not self.is_stop


def _reject(reason, reply_key='unknown'):
    return ValidatedIntent(name='unknown', reply_key=reply_key,
                           rejected_reason=reason)


def _coerce_number(value):
    """Return a finite float, or None. Accepts int/float and clean numeric
    strings (LLMs love quoting numbers); rejects bools, NaN/inf, everything
    else."""
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        v = float(value)
    elif isinstance(value, str):
        try:
            v = float(value.strip())
        except ValueError:
            return None
    else:
        return None
    return v if math.isfinite(v) else None


def _behavior(command_type, data):
    """Render the exact JSON string behavior_ctrl parses."""
    return json.dumps([{"type": command_type, "data": data}])


def validate(raw):
    """Validate one raw intent dict from a brain. Never raises.

    Anything malformed resolves to a spoken refusal (name='unknown' with a
    rejected_reason), never to motion.
    """
    if isinstance(raw, (list, tuple)):
        # Compound command — one thing at a time.
        return _reject('compound_command', reply_key='one_at_a_time')
    if not isinstance(raw, dict):
        return _reject('not_a_dict')

    extra_keys = set(raw) - {'intent', 'params'}
    if extra_keys:
        return _reject(f'unexpected_keys:{sorted(extra_keys)}')

    intent = raw.get('intent')
    if not isinstance(intent, str) or intent not in ALL_INTENTS:
        return _reject(f'unknown_intent:{intent!r}')

    params = raw.get('params', {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        return _reject('params_not_a_dict')

    if intent in NUMERIC_INTENTS:
        return _validate_numeric(intent, params)
    if intent in POINT_INTENTS:
        return _validate_point(intent, params)

    # Remaining intents take no parameters; any supplied are injection-shaped.
    if params:
        return _reject(f'unexpected_params:{sorted(params)}')

    if intent == 'stop':
        return ValidatedIntent(name='stop', behavior_json=_behavior('stop', 0),
                               reply_key='ack_stop', is_stop=True)
    if intent == 'spin_around':
        # Positive angle = counter-clockwise (REP 103), matching behavior_ctrl.
        return ValidatedIntent(name='spin_around',
                               behavior_json=_behavior('spin', 360.0),
                               reply_key='ack_spin_around')
    if intent in DEVICE_INTENTS or intent in SPEECH_INTENTS:
        return ValidatedIntent(name=intent, reply_key=f'ack_{intent}'
                               if intent in DEVICE_INTENTS else intent)

    return _reject(f'unhandled_intent:{intent}')  # unreachable by construction


def _validate_numeric(intent, params):
    pname, default, lo, hi = NUMERIC_INTENTS[intent]
    extra = set(params) - {pname}
    if extra:
        return _reject(f'unexpected_params:{sorted(extra)}')

    raw_value = params.get(pname, default)
    value = _coerce_number(raw_value)
    if value is None:
        return _reject(f'bad_number:{pname}={raw_value!r}')

    notes = []
    reply_key = f'ack_{intent}'
    if value > hi:
        notes.append(f'{pname} {value:g} clamped to max {hi:g}')
        # "That is too far" only makes sense for distances; big turn requests
        # get the angle-clamp template instead.
        reply_key = 'too_far' if pname == 'distance_m' else 'angle_clamped'
        value = hi
    elif value < lo:
        # Covers negatives too: a negative "forward" is a confused brain, not
        # a request to reverse — clamp to the smallest honest motion.
        notes.append(f'{pname} {value:g} clamped to min {lo:g}')
        value = lo

    if pname == 'distance_m':
        data = value
        command = 'drive_on_heading' if intent == 'move_forward' else 'back_up'
    else:
        # behavior_ctrl spin: positive = counter-clockwise = left (REP 103).
        data = value if intent == 'turn_left' else -value
        command = 'spin'

    return ValidatedIntent(name=intent, params={pname: value},
                           behavior_json=_behavior(command, data),
                           reply_key=reply_key, clamp_notes=tuple(notes))


def _validate_point(intent, params):
    extra = set(params) - {'point'}
    if extra:
        return _reject(f'unexpected_params:{sorted(extra)}')
    point = params.get('point')
    if not isinstance(point, str) or point.lower() not in POINT_NAMES:
        return _reject(f'bad_point:{point!r}', reply_key='bad_point')
    point = point.lower()

    if intent == 'save_point':
        return ValidatedIntent(name=intent, params={'point': point},
                               behavior_json=_behavior('save_map_point', point),
                               reply_key='ack_save_point')
    return ValidatedIntent(name=intent, params={'point': point},
                           behavior_json=_behavior('pub_nav_point', point),
                           reply_key='confirm_go_to_point',
                           requires_confirmation=True)


# --- rate limiting --------------------------------------------------------

class CommandRateLimiter:
    """Sliding-window limit on motion commands. Injectable clock for tests."""

    def __init__(self, max_commands=12, window_s=60.0, clock=None):
        import time
        self.max_commands = max_commands
        self.window_s = window_s
        self._clock = clock or time.monotonic
        self._stamps = []

    def allow(self):
        now = self._clock()
        self._stamps = [t for t in self._stamps if now - t < self.window_s]
        if len(self._stamps) >= self.max_commands:
            return False
        self._stamps.append(now)
        return True
