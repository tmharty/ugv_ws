"""Unit tests for the pure chat-session logic (no ROS runtime needed)."""

import pytest

from ugv_voice.chat_session import (ChatHistory, SentenceChunker,
                                    clean_for_tts, is_end_phrase)


# --- SentenceChunker -------------------------------------------------------

def feed_all(chunker, tokens):
    out = []
    for t in tokens:
        out.extend(chunker.feed(t))
    return out


def test_chunker_emits_sentences_at_boundaries():
    c = SentenceChunker()
    got = feed_all(c, ['Hello', ' there.', ' I am', ' a robot.', ' Beep'])
    assert got == ['Hello there.', 'I am a robot.']
    assert c.flush() == 'Beep'


def test_chunker_streamed_token_fragments():
    c = SentenceChunker()
    got = feed_all(c, list('One. Two! Three?'))
    assert got == ['One.', 'Two!']
    assert c.flush() == 'Three?'


def test_chunker_keeps_decimals_intact():
    c = SentenceChunker()
    got = feed_all(c, ['The battery is at 3.5 volts. ', 'Nice.'])
    assert got == ['The battery is at 3.5 volts.']
    assert c.flush() == 'Nice.'


def test_chunker_force_splits_unpunctuated_rambling():
    c = SentenceChunker(max_buffer_chars=40)
    got = feed_all(c, ['word ' * 20])
    assert got  # something was emitted before flush
    assert all(len(p) <= 40 for p in got)
    leftover = c.flush()
    assert (' '.join(got) + ' ' + leftover).split() == ['word'] * 20


def test_chunker_strips_markdown_from_output():
    c = SentenceChunker()
    got = feed_all(c, ['**Hello** `world`. '])
    assert got == ['Hello world.']


def test_chunker_flush_empties_buffer():
    c = SentenceChunker()
    c.feed('partial')
    assert c.flush() == 'partial'
    assert c.flush() == ''


# --- clean_for_tts ---------------------------------------------------------

def test_clean_for_tts_removes_markdown_and_collapses_ws():
    assert clean_for_tts('*Hi*  \n `there` #now') == 'Hi there now'


def test_clean_for_tts_plain_text_unchanged():
    assert clean_for_tts('Just a plain sentence.') == 'Just a plain sentence.'


# --- is_end_phrase ---------------------------------------------------------

def test_end_phrase_exact():
    assert is_end_phrase('goodbye')
    assert is_end_phrase('Goodbye!')
    assert is_end_phrase('end chat')


def test_end_phrase_trailing():
    assert is_end_phrase('okay robot, goodbye')
    assert is_end_phrase("I think that's all")


def test_end_phrase_negatives():
    assert not is_end_phrase('good morning')
    assert not is_end_phrase('tell me about goodbyes in other languages')
    assert not is_end_phrase('')


# --- ChatHistory -----------------------------------------------------------

def test_history_starts_with_system_prompt():
    h = ChatHistory('be a robot')
    h.add_user('hi')
    msgs = h.messages()
    assert msgs[0] == {'role': 'system', 'content': 'be a robot'}
    assert msgs[1] == {'role': 'user', 'content': 'hi'}


def test_history_trims_to_window_keeping_recent():
    h = ChatHistory('sys', max_turns=2)
    for i in range(5):
        h.add_user('q%d' % i)
        h.add_assistant('a%d' % i)
    msgs = h.messages()
    assert len(msgs) == 1 + 2 * 2  # system + 2 turns
    assert msgs[1]['content'] == 'q3'
    assert msgs[-1]['content'] == 'a4'


def test_history_skips_empty_assistant_reply():
    h = ChatHistory('sys')
    h.add_user('hi')
    h.add_assistant('')
    assert h.messages()[-1]['role'] == 'user'


def test_history_clear():
    h = ChatHistory('sys')
    h.add_user('hi')
    h.clear()
    assert h.messages() == [{'role': 'system', 'content': 'sys'}]


def test_history_keeps_tool_messages_with_their_turn():
    h = ChatHistory('sys', max_turns=2)
    for i in range(3):
        h.add_user('q%d' % i)
        h.add_assistant('', [{'function': {'name': 'move', 'arguments': {}}}])
        h.add_tool_result('move', 'Started.')
        h.add_assistant('a%d' % i)
    msgs = h.messages()[1:]
    assert msgs[0] == {'role': 'user', 'content': 'q1'}     # window starts on a user turn
    assert [m['role'] for m in msgs] == ['user', 'assistant', 'tool', 'assistant'] * 2
    assert msgs[1]['tool_calls'] and msgs[1]['content'] == ''
    assert msgs[2] == {'role': 'tool', 'tool_name': 'move', 'content': 'Started.'}


def test_history_skips_reply_with_no_text_and_no_calls():
    h = ChatHistory('sys')
    h.add_user('hi')
    h.add_assistant('', None)
    h.add_assistant(None, [])
    assert h.messages()[-1]['role'] == 'user'


# --- ToolLoop ----------------------------------------------------------------

from ugv_voice.chat_session import ToolLoop, TurnFailed, DEFAULT_SYSTEM_PROMPT
from ugv_voice.tools import ToolCall


def chunks(*tokens, tool_calls=None):
    """Fake Ollama stream: content tokens then an optional tool_calls chunk."""
    out = [{'message': {'role': 'assistant', 'content': t}, 'done': False}
           for t in tokens]
    if tool_calls:
        out.append({'message': {'role': 'assistant', 'content': '',
                                'tool_calls': tool_calls}, 'done': False})
    out.append({'message': {'role': 'assistant', 'content': ''}, 'done': True})
    return out


def call(name, **args):
    return {'function': {'name': name, 'arguments': args}}


class FakeLLM:
    """Scripted replies, one per round; records what it was asked."""

    def __init__(self, rounds):
        self.rounds = list(rounds)
        self.requests = []   # (messages snapshot, tools or None)

    def __call__(self, messages, tools):
        self.requests.append(([dict(m) for m in messages], tools))
        if not self.rounds:
            raise AssertionError('LLM asked for more rounds than scripted')
        r = self.rounds.pop(0)
        if isinstance(r, Exception):
            raise r
        return iter(r)


def run(loop, text, llm, tool_result='ok', still=lambda: True):
    spoken, calls = [], []

    def run_tool(c, spoke):
        calls.append((c, spoke))
        return tool_result
    out = loop.run(text, llm, spoken.append, run_tool, still)
    return out, spoken, calls


def test_loop_prose_only_turn():
    h = ChatHistory('sys')
    llm = FakeLLM([chunks('Hi', ' there.', ' Beep')])
    out, spoken, calls = run(ToolLoop(h), 'hello', llm)
    assert spoken == ['Hi there.', 'Beep'] and out == spoken
    assert calls == []
    assert len(llm.requests) == 1
    assert llm.requests[0][1] is not None            # tools offered
    assert h.messages()[-1] == {'role': 'assistant', 'content': 'Hi there. Beep'}


def test_loop_tool_call_then_narration():
    h = ChatHistory('sys')
    llm = FakeLLM([
        chunks('Rolling forward.', tool_calls=[call('move', direction='forward',
                                                    distance_m=0.3)]),
        chunks('Done, I moved.'),
    ])
    out, spoken, calls = run(ToolLoop(h), 'go forward', llm, tool_result='Started.')
    assert spoken == ['Rolling forward.', 'Done, I moved.']
    assert calls == [(ToolCall('move', {'direction': 'forward', 'distance_m': 0.3}),
                      True)]                          # prose was spoken first
    roles = [m['role'] for m in h.messages()]
    assert roles == ['system', 'user', 'assistant', 'tool', 'assistant']
    tool_msg = h.messages()[3]
    assert tool_msg == {'role': 'tool', 'tool_name': 'move', 'content': 'Started.'}
    # The narration request saw the tool result.
    assert llm.requests[1][0][-1]['role'] == 'tool'


def test_loop_reports_when_model_called_tool_without_speaking():
    h = ChatHistory('sys')
    llm = FakeLLM([chunks(tool_calls=[call('stop')]), chunks('Stopped.')])
    _, _, calls = run(ToolLoop(h), 'stop', llm)
    assert calls[0][1] is False


def test_loop_caps_tool_rounds_and_final_round_has_no_tools():
    h = ChatHistory('sys')
    forever = [chunks('Again.', tool_calls=[call('spin_around')]) for _ in range(10)]
    llm = FakeLLM(forever)
    _, spoken, calls = run(ToolLoop(h, max_tool_rounds=2), 'loop', llm)
    # 2 tool rounds + 1 prose-only round = 3 requests, no more.
    assert len(llm.requests) == 3
    assert llm.requests[0][1] and llm.requests[1][1]
    assert llm.requests[2][1] is None
    # Calls emitted on the tools-less round are still handed over (and
    # will be validated by the node), but no fourth round is requested.
    assert len(calls) == 3
    assert len(llm.rounds) == 7


def test_loop_tools_disabled_sends_none():
    llm = FakeLLM([chunks('ok.')])
    run(ToolLoop(ChatHistory('sys'), tools_enabled=False), 'hi', llm)
    assert llm.requests[0][1] is None


def test_loop_barge_in_stops_speaking_and_tools():
    h = ChatHistory('sys')
    state = {'current': True}
    spoken = []

    def speak(s):
        spoken.append(s)
        state['current'] = False       # stop word lands after sentence one

    llm = FakeLLM([chunks('One.', ' Two.', tool_calls=[call('spin_around')])])
    calls = []
    ToolLoop(h).run('x', llm, speak, lambda c, s: calls.append(c) or 'r',
                    lambda: state['current'])
    assert spoken == ['One.']
    assert calls == []
    assert h.messages()[-1]['role'] == 'user'   # abandoned reply not recorded


def test_loop_transport_failure_raises_turnfailed():
    h = ChatHistory('sys')
    llm = FakeLLM([ConnectionError('refused')])
    with pytest.raises(TurnFailed) as e:
        run(ToolLoop(h), 'go forward', llm)
    assert e.value.spoke_any is False
    assert 'refused' in str(e.value)


def test_loop_midstream_failure_reports_spoke_any():
    h = ChatHistory('sys')

    def broken(messages, tools):
        yield {'message': {'content': 'Hello there. '}, 'done': False}
        raise IOError('socket closed')

    spoken = []
    with pytest.raises(TurnFailed) as e:
        ToolLoop(h).run('hi', broken, spoken.append, lambda c, s: 'r')
    assert spoken == ['Hello there.']
    assert e.value.spoke_any is True


def test_loop_string_arguments_are_parsed():
    h = ChatHistory('sys')
    llm = FakeLLM([chunks(tool_calls=[{'function': {
        'name': 'move', 'arguments': '{"direction": "backward"}'}}]), chunks('ok.')])
    _, _, calls = run(ToolLoop(h), 'back up', llm)
    assert calls[0][0] == ToolCall('move', {'direction': 'backward'})


def test_default_prompt_describes_tools_not_a_ban():
    assert 'cannot move' not in DEFAULT_SYSTEM_PROMPT
    assert 'tool' in DEFAULT_SYSTEM_PROMPT
    assert 'one short sentence' in DEFAULT_SYSTEM_PROMPT
