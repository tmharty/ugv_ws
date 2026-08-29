"""Safety-net tests for the tool registry. Pure Python, no ROS.

Mirrors test_intent_schema.py: every proposed tool call must resolve to
either a clamped, validated intent or a spoken refusal — never to
unclamped motion. Also golden-tests the tool -> Behavior JSON mapping.
"""

import json
import math

import pytest

from ugv_voice import intent_schema, responses, tools
from ugv_voice.tools import (ToolCall, parse_tool_calls, schemas,
                             validate_call, validate_round)


def behavior(v):
    return json.loads(v.intent.behavior_json)


# --- schemas ----------------------------------------------------------------

def test_schema_names_match_registry():
    names = [s['function']['name'] for s in schemas()]
    assert set(names) == tools.TOOL_NAMES
    assert len(names) == len(set(names))
    assert set(names) == {'move', 'turn', 'spin_around', 'stop', 'go_to_point',
                          'save_point', 'battery_status', 'led', 'record_replay'}


def test_schemas_are_fresh_copies():
    a = schemas()
    a[0]['function']['name'] = 'hacked'
    assert schemas()[0]['function']['name'] != 'hacked'


def test_schema_shape_is_ollama_function_tool():
    for s in schemas():
        assert s['type'] == 'function'
        fn = s['function']
        assert fn['name'] in tools._ALLOWED_ARGS
        assert fn['parameters']['type'] == 'object'
        assert set(fn['parameters']['properties']) == tools._ALLOWED_ARGS[fn['name']]
        assert set(fn['parameters']['required']) <= tools._ALLOWED_ARGS[fn['name']]


def test_ack_keys_exist_in_responses():
    for key in tools.ACK_KEYS.values():
        assert key in responses.RESPONSES


# --- golden tool -> Behavior JSON --------------------------------------------

def test_golden_move_forward():
    v = validate_call('move', {'direction': 'forward', 'distance_m': 0.3})
    assert v.intent.behavior_json == '[{"type": "drive_on_heading", "data": 0.3}]'
    assert v.is_motion and not v.rejected
    assert v.ack_key == 'ack_move_forward'
    assert v.result_text == 'Started driving forward 0.3 meters.'


def test_golden_move_backward():
    v = validate_call('move', {'direction': 'backward', 'distance_m': 0.2})
    assert v.intent.behavior_json == '[{"type": "back_up", "data": 0.2}]'


def test_golden_move_default_distance():
    v = validate_call('move', {'direction': 'forward'})
    assert behavior(v)[0]['data'] == 0.3
    v = validate_call('move', {'direction': 'backward'})
    assert behavior(v)[0]['data'] == 0.2


def test_golden_turn_left_positive_right_negative():
    assert validate_call('turn', {'direction': 'left', 'angle_deg': 90}
                         ).intent.behavior_json == '[{"type": "spin", "data": 90.0}]'
    assert validate_call('turn', {'direction': 'right', 'angle_deg': 45}
                         ).intent.behavior_json == '[{"type": "spin", "data": -45.0}]'
    assert behavior(validate_call('turn', {'direction': 'left'}))[0]['data'] == 90.0


def test_golden_spin_around():
    v = validate_call('spin_around', {})
    assert v.intent.behavior_json == '[{"type": "spin", "data": 360.0}]'
    assert v.ack_key == 'ack_spin_around'


def test_golden_stop():
    v = validate_call('stop', None)
    assert v.is_stop
    assert v.intent.behavior_json == '[{"type": "stop", "data": 0}]'


def test_golden_go_to_point_requires_confirmation():
    v = validate_call('go_to_point', {'point': 'A'})
    assert v.intent.behavior_json == '[{"type": "pub_nav_point", "data": "a"}]'
    assert v.intent.requires_confirmation
    assert v.ack_key == 'confirm_go_to_point'
    assert 'confirm' in v.result_text


def test_golden_save_point():
    v = validate_call('save_point', {'point': 'b'})
    assert v.intent.behavior_json == '[{"type": "save_map_point", "data": "b"}]'
    assert not v.intent.requires_confirmation


def test_golden_led_modes():
    for mode in ('on', 'off', 'blink'):
        v = validate_call('led', {'mode': mode})
        assert v.intent.name == 'led_' + mode
        assert v.intent.behavior_json is None
        assert not v.rejected


def test_golden_battery_status():
    v = validate_call('battery_status', {})
    assert v.intent.name == 'battery_status'
    assert not v.rejected and not v.is_motion


# --- clamps ----------------------------------------------------------------

def test_clamp_huge_distance():
    v = validate_call('move', {'direction': 'forward', 'distance_m': 999})
    assert behavior(v)[0]['data'] == 1.0
    assert v.ack_key == 'too_far'          # clamp template overrides the ack
    assert 'reduced' in v.result_text
    assert not v.rejected


def test_clamp_backward_tighter():
    v = validate_call('move', {'direction': 'backward', 'distance_m': 5})
    assert behavior(v)[0]['data'] == 0.5


def test_clamp_negative_distance_never_reverses():
    v = validate_call('move', {'direction': 'forward', 'distance_m': -3})
    assert behavior(v)[0]['type'] == 'drive_on_heading'
    assert behavior(v)[0]['data'] == 0.1


def test_clamp_angle():
    v = validate_call('turn', {'direction': 'right', 'angle_deg': 720})
    assert behavior(v)[0]['data'] == -180.0
    assert v.ack_key == 'angle_clamped'
    assert behavior(validate_call('turn', {'direction': 'left', 'angle_deg': 1}))[0]['data'] == 15.0


def test_numeric_string_accepted_and_clamped():
    v = validate_call('move', {'direction': 'forward', 'distance_m': '12'})
    assert behavior(v)[0]['data'] == 1.0


# --- rejections -----------------------------------------------------------

@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'),
                                 'NaN', 'fast', None, [1], {'v': 1}, True])
def test_bad_numbers_rejected(bad):
    v = validate_call('move', {'direction': 'forward', 'distance_m': bad})
    assert v.rejected
    assert v.intent.behavior_json is None


@pytest.mark.parametrize('name', ['fly', 'self_destruct', 'drive_on_heading',
                                  'spin', '', None, 7, 'MOVE', 'move '])
def test_unknown_tool_rejected(name):
    v = validate_call(name, {})
    assert v.rejected
    assert v.intent.name == 'unknown'
    assert v.intent.behavior_json is None


def test_extra_arguments_are_injection_shaped():
    v = validate_call('move', {'direction': 'forward', 'distance_m': 0.3,
                               'speed': 99})
    assert v.rejected and 'unexpected_arguments' in v.intent.rejected_reason
    v = validate_call('stop', {'really': True})
    assert v.rejected
    v = validate_call('spin_around', {'times': 100})
    assert v.rejected


def test_bad_direction_rejected():
    for d in ('up', 'forwards!', None, 3, ['forward']):
        v = validate_call('move', {'direction': d, 'distance_m': 0.3})
        assert v.rejected, d
    assert validate_call('turn', {'direction': 'around'}).rejected


def test_direction_case_and_whitespace_tolerated():
    v = validate_call('move', {'direction': ' Forward ', 'distance_m': 0.3})
    assert not v.rejected and behavior(v)[0]['type'] == 'drive_on_heading'


def test_arguments_not_object_rejected():
    for args in ('forward', 3, ['forward', 0.3]):
        assert validate_call('move', args).rejected


def test_bad_point_rejected():
    for p in ('z', 'ab', '', None, 1):
        v = validate_call('go_to_point', {'point': p})
        assert v.rejected and v.ack_key == 'bad_point', p


def test_bad_led_mode_rejected():
    for m in ('disco', None, 1, 'ON!'):
        assert validate_call('led', {'mode': m}).rejected


def test_rejections_never_carry_motion():
    """Whatever a rejection looks like, nothing reaches behavior_ctrl."""
    rejected = [
        validate_call('move', {'direction': 'forward', 'distance_m': 'nan'}),
        validate_call('fly', {}),
        validate_call('move', 'forward'),
        validate_call('turn', {'direction': 'left', 'angle_deg': 90, 'x': 1}),
    ]
    for v in rejected:
        assert v.rejected
        assert v.intent.behavior_json is None
        assert not v.is_motion and not v.is_stop
        assert v.ack_key in responses.RESPONSES
        assert v.result_text.startswith('Refused')


# --- compound / per-round ------------------------------------------------

def test_round_allows_one_call_and_refuses_the_rest():
    calls = [ToolCall('move', {'direction': 'forward', 'distance_m': 0.3}),
             ToolCall('turn', {'direction': 'left'}),
             ToolCall('spin_around', {})]
    out = validate_round(calls)
    assert len(out) == 3
    assert not out[0].rejected and out[0].is_motion
    assert out[1].rejected and out[1].ack_key == 'one_at_a_time'
    assert out[2].rejected and out[2].intent.behavior_json is None


def test_round_empty():
    assert validate_round([]) == []


# --- parse_tool_calls (Ollama message shapes) ----------------------------

def test_parse_dict_arguments():
    msg = {'role': 'assistant', 'content': '',
           'tool_calls': [{'function': {'name': 'move',
                                        'arguments': {'direction': 'forward',
                                                      'distance_m': 0.5}}}]}
    calls = parse_tool_calls(msg)
    assert calls == [ToolCall('move', {'direction': 'forward', 'distance_m': 0.5})]


def test_parse_json_string_arguments():
    msg = {'tool_calls': [{'function': {
        'name': 'turn', 'arguments': '{"direction": "left", "angle_deg": 30}'}}]}
    assert parse_tool_calls(msg) == [ToolCall('turn', {'direction': 'left',
                                                       'angle_deg': 30})]


def test_parse_empty_string_arguments_is_empty_dict():
    msg = {'tool_calls': [{'function': {'name': 'stop', 'arguments': ''}}]}
    assert parse_tool_calls(msg) == [ToolCall('stop', {})]


def test_parse_garbage_arguments_lead_to_refusal():
    msg = {'tool_calls': [{'function': {'name': 'move', 'arguments': '{oops'}}]}
    (call,) = parse_tool_calls(msg)
    assert validate_call(call.name, call.arguments).rejected


def test_parse_malformed_entries_are_refused_not_dropped():
    msg = {'tool_calls': [7, {'function': 'move'}, {'function': {'name': 5}}]}
    calls = parse_tool_calls(msg)
    assert len(calls) == 3
    for c in calls:
        assert validate_call(c.name, c.arguments).rejected


def test_parse_no_tool_calls():
    assert parse_tool_calls({'content': 'hi'}) == []
    assert parse_tool_calls({'tool_calls': None}) == []
    assert parse_tool_calls('not a dict') == []


def test_parse_tool_calls_not_a_list_is_refused():
    (call,) = parse_tool_calls({'tool_calls': {'function': {'name': 'move'}}})
    assert validate_call(call.name, call.arguments).rejected


# --- consistency with intent_schema --------------------------------------

def test_tool_clamps_are_intent_schema_clamps():
    """The registry adds no second clamp table; limits come from intent_schema."""
    _, _, lo, hi = intent_schema.NUMERIC_INTENTS['move_forward']
    assert behavior(validate_call('move', {'direction': 'forward',
                                           'distance_m': hi * 10}))[0]['data'] == hi
    assert behavior(validate_call('move', {'direction': 'forward',
                                           'distance_m': lo / 10}))[0]['data'] == lo
    _, _, lo, hi = intent_schema.NUMERIC_INTENTS['turn_left']
    assert behavior(validate_call('turn', {'direction': 'left',
                                           'angle_deg': hi * 10}))[0]['data'] == hi


def test_every_accepted_motion_is_within_envelope():
    """Fuzz-ish sweep: whatever number comes in, what goes out is bounded."""
    values = [-1e9, -1, 0, 1e-9, 0.05, 0.1, 0.33, 0.5, 1, 1.01, 2, 1e9,
              '0.7', '1e3', ' 2 ']
    for v in values:
        out = validate_call('move', {'direction': 'forward', 'distance_m': v})
        assert 0.1 <= behavior(out)[0]['data'] <= 1.0, v
        out = validate_call('move', {'direction': 'backward', 'distance_m': v})
        assert 0.1 <= behavior(out)[0]['data'] <= 0.5, v
        out = validate_call('turn', {'direction': 'right', 'angle_deg': v})
        assert 15.0 <= abs(behavior(out)[0]['data']) <= 180.0, v
        assert math.isfinite(behavior(out)[0]['data'])


# --- record_replay (Phase 3) ----------------------------------------------

def test_record_replay_in_schema_and_defaults():
    assert 'record_replay' in tools.TOOL_NAMES
    v = validate_call('record_replay', {})
    assert not v.rejected and not v.is_motion and not v.is_stop
    assert v.intent.behavior_json is None              # never touches behavior_ctrl
    assert v.intent.params == {'duration_s': 5.0, 'speeds': [1.0]}
    assert 'Recorded 5 seconds' in v.result_text


def test_record_replay_clamps_duration():
    assert validate_call('record_replay', {'duration_s': 60}).intent.params['duration_s'] == 15.0
    assert validate_call('record_replay', {'duration_s': 0.5}).intent.params['duration_s'] == 3.0
    v = validate_call('record_replay', {'duration_s': 999})
    assert v.intent.clamp_notes and 'reduced' in v.result_text
    assert validate_call('record_replay', {'duration_s': '10'}).intent.params['duration_s'] == 10.0


def test_record_replay_clamps_speeds_and_count():
    v = validate_call('record_replay', {'speeds': [0.1, 9, 1.3, 0.7, 1.0, 2.0]})
    assert v.intent.params['speeds'] == [0.5, 2.0, 1.3, 0.7]   # clamped, capped at 4
    assert len(v.intent.clamp_notes) == 3
    assert validate_call('record_replay', {'speeds': 1.3}).intent.params['speeds'] == [1.3]
    assert validate_call('record_replay', {'speeds': []}).intent.params['speeds'] == [1.0]
    assert validate_call('record_replay', {'speeds': None}).intent.params['speeds'] == [1.0]


def test_record_replay_named_speeds():
    v = validate_call('record_replay', {'speeds': ['deep', 'normal', 'chipmunk']})
    assert v.intent.params['speeds'] == [0.7, 1.0, 1.3]
    assert validate_call('record_replay', {'speeds': 'Chipmunk'}).intent.params['speeds'] == [1.3]


@pytest.mark.parametrize('args', [
    {'duration_s': 'nan'}, {'duration_s': None}, {'duration_s': [5]},
    {'speeds': ['warp']}, {'speeds': [float('inf')]}, {'speeds': {'a': 1}},
    {'speeds': [True]}, {'duration_s': 5, 'loud': True},
])
def test_record_replay_rejections(args):
    v = validate_call('record_replay', args)
    assert v.rejected
    assert v.intent.behavior_json is None


# --- record_replay request gate (issue: "tell me a joke" fired the tool) -----

@pytest.mark.parametrize('text', [
    'Can you record my voice?',
    'record me and play it back like a chipmunk',
    'Say it back to me, please.',
    'Repeat after me: hello robot',
    'I want to hear myself',
])
def test_record_requested_true(text):
    assert tools.record_requested(text)


@pytest.mark.parametrize('text', [
    'Can you tell me a joke?',
    'Tell me a funny story',
    'Sing a song',
    'What is your battery at?',
    '',
    None,
])
def test_record_requested_false(text):
    assert not tools.record_requested(text)


def test_record_replay_refused_when_not_requested_is_silent():
    v = validate_call('record_replay', {'duration_s': 5},
                      user_text='Can you tell me a joke?')
    assert v.rejected
    assert v.intent.rejected_reason == 'not_requested'
    assert v.ack_key is None                     # nothing is spoken
    assert 'without calling any tool' in v.result_text
    # validate_round passes the utterance through
    r = tools.validate_round([tools.ToolCall('record_replay', {})],
                             user_text='tell me a joke')
    assert r[0].rejected and r[0].intent.rejected_reason == 'not_requested'


def test_record_replay_allowed_when_requested_or_text_unknown():
    assert not validate_call('record_replay', {}, user_text='record my voice').rejected
    assert not validate_call('record_replay', {}).rejected   # no text = no gate


def test_record_replay_description_is_strict():
    desc = next(s for s in tools.TOOL_SCHEMAS
                if s['function']['name'] == 'record_replay')['function']['description']
    assert 'ONLY' in desc and 'jokes' in desc
    assert 'funny' not in desc
