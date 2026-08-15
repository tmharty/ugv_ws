"""Keyword/regex brain: deterministic transcript -> intent mapping.

Deliberately conservative: an utterance that doesn't clearly match a pattern
becomes 'unknown' (spoken refusal) rather than a guessed motion. Numbers are
parsed from digits or number words ("three", "half"); units meters/centimeters
for distance, degrees for turns. Missing numbers fall back to the validator's
per-intent defaults.
"""

import re

from ..intent_schema import contains_stop_word
from . import Brain

_NUMBER_WORDS = {
    'zero': 0, 'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
    'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10, 'eleven': 11,
    'twelve': 12, 'thirteen': 13, 'fourteen': 14, 'fifteen': 15,
    'sixteen': 16, 'seventeen': 17, 'eighteen': 18, 'nineteen': 19,
    'twenty': 20, 'thirty': 30, 'forty': 40, 'fifty': 50, 'sixty': 60,
    'seventy': 70, 'eighty': 80, 'ninety': 90, 'hundred': 100,
    'half': 0.5, 'quarter': 0.25,
}

_AFFIRM_RE = re.compile(
    r'\b(yes|yeah|yep|yup|sure|okay|ok|affirmative|do it|go for it|'
    r'go ahead|confirm)\b')
_DENY_RE = re.compile(r'\b(no|nope|nah|negative|cancel|never mind|nevermind)\b')

# Bare "forward…" only counts at the start of the utterance ("forward two
# meters"); mid-sentence mentions need a command verb, so narration like
# "the bus went forward" cannot move the robot.
_FORWARD_RE = re.compile(
    r'\b(go|move|drive|roll|scoot|come)\b.*\b(forward|forwards|ahead|straight)\b'
    r'|^(forward|forwards|ahead)\b')
_BACKWARD_RE = re.compile(
    r'\b(back up|back it up|reverse|go back|move back|drive back|'
    r'backward|backwards)\b')
_LEFT_RE = re.compile(r'\b(turn|rotate|spin|look)\b.*\bleft\b')
_RIGHT_RE = re.compile(r'\b(turn|rotate|spin|look)\b.*\bright\b')
_TURN_AROUND_RE = re.compile(r'\bturn around\b')
_SPIN_RE = re.compile(r'\b(spin|twirl|do a spin|spin around|dance)\b')

_GO_TO_POINT_RE = re.compile(r'\bgo to( point)? ([a-g])\b')
# Direct form ("save point d") first — with the descriptive form first, it
# would match with no letter captured and shadow the direct form.
_SAVE_POINT_RE = re.compile(
    r'\b(save|remember|mark)\b\s+(?:point\s+)?([a-g])\b'
    r'|\b(save|remember|mark)\b.*?\b(point|spot|place|here|this)\b'
    r'(?:.*?\b(?:as|point)\s+([a-g])\b)?')

_BATTERY_RE = re.compile(r'\b(battery|power level|charge|voltage)\b')
_LED_RE = re.compile(r'\b(light|lights|led|leds|lamp)\b')
_ON_RE = re.compile(r'\bon\b')
_OFF_RE = re.compile(r'\boff\b')
_BLINK_RE = re.compile(r'\b(blink|flash|strobe)\b')

_GREETING_RE = re.compile(r'^(hi|hello|hey|howdy|good (morning|afternoon|evening))\b')
_WHO_RE = re.compile(r"\b(who are you|what are you|what('s| is) your name)\b")
_HELP_RE = re.compile(r'\bwhat can you do\b|\bhelp\b|\bcommands?\b')


def _normalize(text):
    text = text.lower()
    text = re.sub(r"[^a-z0-9' ]+", ' ', text)
    return re.sub(r'\s+', ' ', text).strip()


def _extract_number(text):
    """First number in the utterance, from digits or number words.
    Handles simple compounds ("twenty five") and "X and a half"."""
    m = re.search(r'\d+(\.\d+)?', text)
    if m:
        value = float(m.group(0))
    else:
        value = None
        tokens = text.split()
        for i, tok in enumerate(tokens):
            if tok in _NUMBER_WORDS:
                value = float(_NUMBER_WORDS[tok])
                if (value >= 20 and i + 1 < len(tokens)
                        and tokens[i + 1] in _NUMBER_WORDS
                        and _NUMBER_WORDS[tokens[i + 1]] < 10):
                    value += _NUMBER_WORDS[tokens[i + 1]]
                break
        if value is None:
            return None
    if re.search(r'\band a half\b', text):
        value += 0.5
    return value


def _extract_distance(text):
    """Distance in meters, or None to use the intent default."""
    if re.search(r'\b(a (little|tiny|wee)( bit)?|a bit|slightly)\b', text):
        return 0.1
    value = _extract_number(text)
    if value is None:
        return None
    if re.search(r'\b(centimeter|centimetre|cm)s?\b', text):
        return value / 100.0
    return value  # unit-less numbers are meters


def _extract_angle(text):
    return _extract_number(text)  # degrees; None -> validator default (90)


class RuleBrain(Brain):
    name = 'rules'

    def parse(self, text):
        t = _normalize(text)
        if not t:
            return {'intent': 'unknown'}

        # Order matters: stop outranks everything, and specific patterns
        # (turn around, go to point) come before the general ones they overlap.
        if contains_stop_word(t):
            return {'intent': 'stop'}

        m = _GO_TO_POINT_RE.search(t)
        if m:
            return {'intent': 'go_to_point', 'params': {'point': m.group(2)}}
        m = _SAVE_POINT_RE.search(t)
        if m:
            point = m.group(2) or m.group(5)
            if point:
                return {'intent': 'save_point', 'params': {'point': point}}
            return {'intent': 'unknown'}  # "save this spot" without a letter

        if _TURN_AROUND_RE.search(t):
            return {'intent': 'turn_left', 'params': {'angle_deg': 180}}
        if _LEFT_RE.search(t):
            return self._turn('turn_left', t)
        if _RIGHT_RE.search(t):
            return self._turn('turn_right', t)
        if _SPIN_RE.search(t):
            return {'intent': 'spin_around'}

        if _BACKWARD_RE.search(t):
            return self._move('move_backward', t)
        if _FORWARD_RE.search(t):
            return self._move('move_forward', t)

        if _BATTERY_RE.search(t):
            return {'intent': 'battery_status'}
        if _LED_RE.search(t):
            if _BLINK_RE.search(t):
                return {'intent': 'led_blink'}
            if _OFF_RE.search(t):
                return {'intent': 'led_off'}
            if _ON_RE.search(t):
                return {'intent': 'led_on'}
            return {'intent': 'unknown'}

        if _WHO_RE.search(t):
            return {'intent': 'who_are_you'}
        if _HELP_RE.search(t):
            return {'intent': 'what_can_you_do'}
        if _GREETING_RE.search(t):
            return {'intent': 'greeting'}

        if _AFFIRM_RE.search(t):
            return {'intent': 'affirm'}
        if _DENY_RE.search(t):
            return {'intent': 'deny'}

        return {'intent': 'unknown'}

    @staticmethod
    def _move(intent, t):
        distance = _extract_distance(t)
        if distance is None:
            return {'intent': intent}
        return {'intent': intent, 'params': {'distance_m': distance}}

    @staticmethod
    def _turn(intent, t):
        angle = _extract_angle(t)
        if angle is None:
            return {'intent': intent}
        return {'intent': intent, 'params': {'angle_deg': angle}}
