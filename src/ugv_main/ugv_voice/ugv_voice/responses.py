"""Scripted lines: the deterministic part of what the robot says.

In the scripted pipeline (voice.launch.py safe mode) every spoken line
comes from this table. In chat-with-tools the LLM speaks freely and these
templates survive only where determinism matters: safety messages,
motion acknowledgements/clamps, refusals and offline/fallback lines.
Persona: clearly a machine. Short, cheerful, robotic.

Keys marked (safety) are spoken at PRIORITY_SAFETY and interrupt playback.
"""

RESPONSES = {
    # Motion acknowledgements (spoken BEFORE the motion starts — the speech
    # itself is the human reaction window).
    'ack_move_forward':  ['Affirmative. Rolling forward {distance_m:g} meters. Beep.'],
    'ack_move_backward': ['Affirmative. Backing up {distance_m:g} meters. Beep.'],
    'ack_turn_left':     ['Turning left {angle_deg:g} degrees.'],
    'ack_turn_right':    ['Turning right {angle_deg:g} degrees.'],
    'ack_spin_around':   ['Commencing spin. Wheee. Beep.'],
    'ack_stop':          ['Stopping.'],                     # (safety)
    'estop':             ['Stopping.'],                     # (safety)

    # Clamps and refusals.
    'too_far':       ['That is too far for one command. '
                      'I will go {distance_m:g} meters instead.'],
    'angle_clamped': ['That is too much turning. '
                      'I will turn {angle_deg:g} degrees instead.'],
    'one_at_a_time': ['One thing at a time, please. Beep.'],
    'unknown':       ['Command not recognized. '
                      'Say: what can you do? For my command list.'],
    'rate_limited':  ['Too many commands. My circuits need a moment.'],
    'heard_nothing': ['I did not hear anything. Beep.'],

    # Navigation to saved points.
    'confirm_go_to_point': ['Shall I drive to point {point}? Say yes to confirm.'],
    'nav_confirmed':       ['Affirmative. Navigating to point {point}.'],
    'nav_cancelled':       ['Navigation cancelled.'],
    'ack_save_point':      ['Point {point} saved to my memory banks.'],
    'bad_point':           ['I only know points A through G.'],

    # Small talk (scripted).
    'greeting':        ['Hello human. Robot ready. Beep.'],
    'who_are_you':     ['I am the U G V Beast. A tracked robot. Beep boop.'],
    'what_can_you_do': ['I can drive forward and backward, turn, spin around, '
                        'go to saved points, blink my lights, and report my '
                        'battery. Say stop at any time and I will stop.'],
    'affirm':          ['There is nothing to confirm. Beep.'],
    'deny':            ['Okay. Doing nothing.'],

    # Status and devices.
    'battery_status':         ['My battery is at {voltage:.1f} volts.'],
    'battery_status_unknown': ['I cannot read my battery right now.'],
    'ack_led_on':    ['Lights on.'],
    'ack_led_off':   ['Lights off.'],
    'ack_led_blink': ['Blinking my lights. Beep beep.'],

    # System messages.
    'not_ready': ['My motion system is not responding. I will not move.'],

    # Chat-with-tools fallback lines (chat_node).
    'chat_offline': ['My chat brain is not answering. '
                     'I can still do simple commands.'],
    'chat_error':   ['My chat brain hit an error. Say that again.'],
}

_counters = {}


def render(key, variant=None, **slots):
    """Render a response by key. Unknown keys fall back to 'unknown' so a
    template typo can never silence the robot. Cycles variants unless one is
    pinned (tests pin variant=0)."""
    variants = RESPONSES.get(key) or RESPONSES['unknown']
    if variant is None:
        variant = _counters.get(key, 0)
        _counters[key] = variant + 1
    text = variants[variant % len(variants)]
    try:
        return text.format(**slots)
    except (KeyError, IndexError, ValueError):
        # A missing slot must never crash the speech path.
        return text.replace('{', ' ').replace('}', ' ')
