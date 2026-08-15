"""Utterance -> intent table for the rule brain, including near-misses."""

import pytest

from ugv_voice.brains.rule_brain import RuleBrain

brain = RuleBrain()


CASES = [
    # motion
    ('go forward', {'intent': 'move_forward'}),
    ('move forward half a meter',
     {'intent': 'move_forward', 'params': {'distance_m': 0.5}}),
    ('drive ahead 2 meters',
     {'intent': 'move_forward', 'params': {'distance_m': 2.0}}),  # validator clamps
    ('scoot ahead a tiny bit',
     {'intent': 'move_forward', 'params': {'distance_m': 0.1}}),
    ('go forward thirty centimeters',
     {'intent': 'move_forward', 'params': {'distance_m': 0.3}}),
    ('back up', {'intent': 'move_backward'}),
    ('reverse a little bit',
     {'intent': 'move_backward', 'params': {'distance_m': 0.1}}),
    ('go back one meter',
     {'intent': 'move_backward', 'params': {'distance_m': 1.0}}),  # clamped later
    ('turn left', {'intent': 'turn_left'}),
    ('turn left ninety degrees',
     {'intent': 'turn_left', 'params': {'angle_deg': 90.0}}),
    ('turn right 45 degrees',
     {'intent': 'turn_right', 'params': {'angle_deg': 45.0}}),
    ('turn around', {'intent': 'turn_left', 'params': {'angle_deg': 180}}),
    ('spin around', {'intent': 'spin_around'}),
    ('do a spin', {'intent': 'spin_around'}),
    # stop outranks everything
    ('stop', {'intent': 'stop'}),
    ('stop stop stop', {'intent': 'stop'}),
    ('whoa stop right there', {'intent': 'stop'}),
    ('go forward and then stop', {'intent': 'stop'}),
    # points
    ('go to point a', {'intent': 'go_to_point', 'params': {'point': 'a'}}),
    ('go to b', {'intent': 'go_to_point', 'params': {'point': 'b'}}),
    ('save this spot as c', {'intent': 'save_point', 'params': {'point': 'c'}}),
    ('remember this place as g',
     {'intent': 'save_point', 'params': {'point': 'g'}}),
    ('save point d', {'intent': 'save_point', 'params': {'point': 'd'}}),
    # status / devices
    ('how is your battery', {'intent': 'battery_status'}),
    ('what is your voltage', {'intent': 'battery_status'}),
    ('turn the lights on', {'intent': 'led_on'}),
    ('lights off', {'intent': 'led_off'}),
    ('blink your lights', {'intent': 'led_blink'}),
    # small talk
    ('hello', {'intent': 'greeting'}),
    ('hey there robot', {'intent': 'greeting'}),
    ('who are you', {'intent': 'who_are_you'}),
    ("what's your name", {'intent': 'who_are_you'}),
    ('what can you do', {'intent': 'what_can_you_do'}),
    # confirmation words
    ('yes', {'intent': 'affirm'}),
    ('yes please', {'intent': 'affirm'}),
    ('no', {'intent': 'deny'}),
    ('cancel', {'intent': 'deny'}),
    # near-misses must NOT trigger motion
    ('go for it', {'intent': 'affirm'}),
    ('that was fast', {'intent': 'unknown'}),
    ('the bus went forward without us', {'intent': 'unknown'}),
    ('forward one meter', {'intent': 'move_forward', 'params': {'distance_m': 1.0}}),
    ('what a nice day', {'intent': 'unknown'}),
    ('', {'intent': 'unknown'}),
    ('asdf qwerty', {'intent': 'unknown'}),
]


@pytest.mark.parametrize('text,expected', CASES,
                         ids=[c[0] or '<empty>' for c in CASES])
def test_utterance_mapping(text, expected):
    assert brain.parse(text) == expected


def test_parse_never_returns_non_dict():
    for text in ('', '   ', '!!!', '123', 'ñ ü ø'):
        out = brain.parse(text)
        assert isinstance(out, dict) and 'intent' in out
