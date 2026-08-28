"""Tool registry: what the LLM may call, and the validator every call passes.

This is the chat-with-tools counterpart of intent_schema.py, and it reuses
it. Each tool is:

  * a JSON schema — the ``tools`` entry sent to Ollama's /api/chat, and
  * a mapping from (tool name, arguments) onto a raw intent dict, which
    then goes through ``intent_schema.validate`` — the same allow-list and
    clamps the scripted pipeline uses. Nothing here adds a second clamp
    implementation; nothing here bypasses the first.

The LLM only ever *proposes* a call. Unknown names, malformed arguments,
NaN/negative/huge numbers, extra keys and compound payloads all resolve to
a ``ValidatedIntent`` carrying a ``rejected_reason`` (spoken refusal) and
never to motion. Pure Python: zero ROS imports, fully unit-testable.
"""

import json
from dataclasses import dataclass
from . import intent_schema
from .intent_schema import POINT_NAMES, ValidatedIntent

# Per-tool-round cap on the number of calls we will even look at; anything
# beyond it is dropped with a refusal. One utterance is one action.
MAX_CALLS_PER_ROUND = 1


def _schema(name, description, properties=None, required=()):
    return {
        'type': 'function',
        'function': {
            'name': name,
            'description': description,
            'parameters': {
                'type': 'object',
                'properties': properties or {},
                'required': list(required),
            },
        },
    }


# Descriptions are written for a small model: say what the tool does, the
# honest limits, and when to use it. The clamps are stated so the model
# proposes sane values, but the validator does not trust it to.
TOOL_SCHEMAS = [
    _schema(
        'move',
        'Drive the robot in a straight line a short distance, slowly. '
        'Forward 0.1 to 1.0 meters, backward 0.1 to 0.5 meters. Use for '
        '"go forward", "back up", "scoot ahead a bit".',
        {
            'direction': {'type': 'string', 'enum': ['forward', 'backward']},
            'distance_m': {'type': 'number',
                           'description': 'Distance in meters. "A little" is 0.1.'},
        },
        ('direction',),
    ),
    _schema(
        'turn',
        'Turn the robot in place, 15 to 180 degrees. Use for "turn left", '
        '"look right", "turn around" (180).',
        {
            'direction': {'type': 'string', 'enum': ['left', 'right']},
            'angle_deg': {'type': 'number',
                          'description': 'Angle in degrees. Default 90.'},
        },
        ('direction',),
    ),
    _schema('spin_around',
            'Spin a full circle in place. Use for "spin", "do a dance turn".'),
    _schema('stop',
            'Stop all motion immediately. Use whenever the user wants the '
            'robot to stop, halt, freeze or stay still.'),
    _schema(
        'go_to_point',
        'Navigate to a saved map point named A to G. The user will be asked '
        'to say yes before the robot goes.',
        {'point': {'type': 'string', 'enum': [p.upper() for p in POINT_NAMES]}},
        ('point',),
    ),
    _schema(
        'save_point',
        'Save the robot\'s current position as a map point named A to G.',
        {'point': {'type': 'string', 'enum': [p.upper() for p in POINT_NAMES]}},
        ('point',),
    ),
    _schema('battery_status',
            'Read the battery voltage. Use when asked about battery, charge '
            'or power.'),
    _schema(
        'led',
        'Control the robot\'s lights: turn them on, off, or blink them.',
        {'mode': {'type': 'string', 'enum': ['on', 'off', 'blink']}},
        ('mode',),
    ),
]

TOOL_NAMES = frozenset(s['function']['name'] for s in TOOL_SCHEMAS)

# Allowed argument keys per tool. Extra keys are injection-shaped → reject.
_ALLOWED_ARGS = {
    'move': {'direction', 'distance_m'},
    'turn': {'direction', 'angle_deg'},
    'spin_around': set(),
    'stop': set(),
    'go_to_point': {'point'},
    'save_point': {'point'},
    'battery_status': set(),
    'led': {'mode'},
}

# Short spoken acknowledgement per validated intent, used when the model
# calls a motion tool without saying anything first (speak-then-act needs
# *something* audible before wheels move). Keys are responses.py entries.
ACK_KEYS = {
    'move_forward': 'ack_move_forward',
    'move_backward': 'ack_move_backward',
    'turn_left': 'ack_turn_left',
    'turn_right': 'ack_turn_right',
    'spin_around': 'ack_spin_around',
    'save_point': 'ack_save_point',
    'go_to_point': 'confirm_go_to_point',
}


@dataclass(frozen=True)
class ToolCall:
    """One proposed call, as parsed from an Ollama message."""
    name: str
    arguments: object   # whatever the model sent; validated later


@dataclass(frozen=True)
class ValidatedTool:
    """A tool call after validation — the only thing chat_node dispatches."""
    tool: str                     # requested tool name (may be unknown)
    intent: ValidatedIntent       # intent_schema result; motion lives here
    result_text: str              # fed back to the model as the tool result

    @property
    def rejected(self):
        return self.intent.rejected_reason is not None

    @property
    def is_motion(self):
        return self.intent.is_motion

    @property
    def is_stop(self):
        return self.intent.is_stop

    @property
    def ack_key(self):
        """responses.py key to speak before acting, or None."""
        if self.rejected:
            return self.intent.reply_key
        if self.intent.clamp_notes:
            return self.intent.reply_key   # too_far / angle_clamped
        return ACK_KEYS.get(self.intent.name)


def schemas():
    """Tool list for the Ollama request (a fresh copy each time)."""
    return json.loads(json.dumps(TOOL_SCHEMAS))


def parse_tool_calls(message):
    """Extract ToolCalls from an Ollama assistant message dict.

    Tolerant of the shapes different Ollama versions emit (arguments as a
    dict or as a JSON string; missing 'function' wrapper). Never raises;
    unparseable entries become calls with a sentinel name so they are
    refused downstream rather than silently dropped.
    """
    calls = []
    if not isinstance(message, dict):
        return calls
    raw_calls = message.get('tool_calls') or []
    if not isinstance(raw_calls, list):
        return [ToolCall(name='<malformed>', arguments=None)]
    for entry in raw_calls:
        fn = entry.get('function', entry) if isinstance(entry, dict) else None
        if not isinstance(fn, dict):
            calls.append(ToolCall(name='<malformed>', arguments=None))
            continue
        name = fn.get('name')
        args = fn.get('arguments', {})
        if isinstance(args, str):
            try:
                args = json.loads(args) if args.strip() else {}
            except ValueError:
                args = None   # rejected by validate_call
        calls.append(ToolCall(name=name if isinstance(name, str) else '<malformed>',
                              arguments=args))
    return calls


def _reject(tool, reason, reply_key='unknown', result_text=None):
    intent = ValidatedIntent(name='unknown', reply_key=reply_key,
                             rejected_reason=reason)
    return ValidatedTool(tool=str(tool), intent=intent,
                         result_text=result_text or
                         'Refused: %s. Tell the user you cannot do that.' % reason)


def validate_call(name, arguments):
    """Validate one proposed tool call. Never raises.

    Returns a ValidatedTool whose ``intent`` is the intent_schema result.
    Anything malformed carries a rejected_reason and no behavior_json.
    """
    if not isinstance(name, str) or name not in TOOL_NAMES:
        return _reject(name, 'unknown_tool:%r' % (name,))
    if arguments is None:
        arguments = {}
    if not isinstance(arguments, dict):
        return _reject(name, 'arguments_not_an_object')
    extra = set(arguments) - _ALLOWED_ARGS[name]
    if extra:
        return _reject(name, 'unexpected_arguments:%s' % sorted(extra))

    raw = _to_raw_intent(name, arguments)
    if isinstance(raw, ValidatedTool):
        return raw                                  # already a rejection
    intent = intent_schema.validate(raw)
    return ValidatedTool(tool=name, intent=intent,
                         result_text=_result_text(name, intent))


def _direction(name, arguments, choices):
    d = arguments.get('direction')
    if not isinstance(d, str) or d.lower().strip() not in choices:
        return None
    return d.lower().strip()


def _to_raw_intent(name, arguments):
    """Map (tool, args) → intent_schema raw dict, or a rejection."""
    if name == 'move':
        d = _direction(name, arguments, ('forward', 'backward'))
        if d is None:
            return _reject(name, 'bad_direction:%r' % (arguments.get('direction'),))
        params = {}
        if 'distance_m' in arguments:
            params['distance_m'] = arguments['distance_m']
        return {'intent': 'move_forward' if d == 'forward' else 'move_backward',
                'params': params}
    if name == 'turn':
        d = _direction(name, arguments, ('left', 'right'))
        if d is None:
            return _reject(name, 'bad_direction:%r' % (arguments.get('direction'),))
        params = {}
        if 'angle_deg' in arguments:
            params['angle_deg'] = arguments['angle_deg']
        return {'intent': 'turn_left' if d == 'left' else 'turn_right',
                'params': params}
    if name in ('go_to_point', 'save_point'):
        return {'intent': name, 'params': {'point': arguments.get('point')}}
    if name == 'led':
        mode = arguments.get('mode')
        if not isinstance(mode, str) or mode.lower().strip() not in ('on', 'off', 'blink'):
            return _reject(name, 'bad_led_mode:%r' % (mode,))
        return {'intent': 'led_' + mode.lower().strip()}
    # Parameterless tools: spin_around, stop, battery_status.
    return {'intent': name}


def _result_text(name, intent):
    """What the model is told after dispatch (it narrates this)."""
    if intent.rejected_reason:
        if intent.reply_key == 'bad_point':
            return 'Refused: only points A through G exist.'
        return 'Refused: %s. Tell the user you cannot do that.' % intent.rejected_reason
    p = intent.params
    if intent.name == 'move_forward':
        text = 'Started driving forward %g meters.' % p['distance_m']
    elif intent.name == 'move_backward':
        text = 'Started backing up %g meters.' % p['distance_m']
    elif intent.name in ('turn_left', 'turn_right'):
        text = 'Started turning %s %g degrees.' % (
            intent.name.split('_')[1], p['angle_deg'])
    elif intent.name == 'spin_around':
        text = 'Started a full spin.'
    elif intent.name == 'stop':
        text = 'Stopped.'
    elif intent.name == 'go_to_point':
        text = ('Asked the user to confirm navigation to point %s. Do not '
                'claim to be moving yet.' % p['point'].upper())
    elif intent.name == 'save_point':
        text = 'Saved current position as point %s.' % p['point'].upper()
    elif intent.name.startswith('led_'):
        text = 'Lights %s.' % intent.name[4:]
    elif intent.name == 'battery_status':
        text = 'Battery reading follows.'   # chat_node fills in the voltage
    else:
        text = 'Done.'
    if intent.clamp_notes:
        text += ' (Request was reduced to the safe limit: %s.)' % '; '.join(
            intent.clamp_notes)
    return text


def validate_round(tool_calls):
    """Validate a whole tool_calls list from one model message.

    Only the first MAX_CALLS_PER_ROUND calls are honoured; the rest are
    refused as compound commands (one thing at a time).
    """
    out = []
    for i, call in enumerate(tool_calls):
        if i >= MAX_CALLS_PER_ROUND:
            out.append(_reject(call.name, 'compound_command',
                               reply_key='one_at_a_time',
                               result_text='Refused: one action per request.'))
            continue
        out.append(validate_call(call.name, call.arguments))
    return out
