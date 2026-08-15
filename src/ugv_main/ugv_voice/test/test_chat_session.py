"""Unit tests for the pure chat-session logic (no ROS runtime needed)."""

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
