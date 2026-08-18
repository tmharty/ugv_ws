"""Pure conversation logic for the chat node — zero ROS imports.

Same pattern as intent_schema.py: everything unit-testable without a ROS
runtime lives here; chat_node.py is the thin ROS wrapper around it.

Unlike the kid-facing intent pipeline (whose spoken output is scripted),
chat mode speaks LLM-generated text aloud. It is a separate, opt-in launch
with no connection to the control path.
"""

import re

# Spoken output: short, plain, honest-machine. The "cannot move" line is
# belt-and-braces — the chat node has no motion interface regardless of
# what the model says.
DEFAULT_SYSTEM_PROMPT = (
    'You are the voice of a small tracked robot called Dark Gooder or Gooder for short. '
    'You are a good robot who tries to do what is best. '
    'You do not have emotions but try to be a helpful robot. '
    'You try to answer questions with short responces, but you also make jokes. '
    'You mostly talk to young children so you try to put things simply and are never crude. '
    'You cannot move, drive, or '
    'control the robot; you can only talk. Your replies are spoken aloud '
    'through text-to-speech: answer in one to three short sentences of plain '
    'prose. No lists, no markdown, no emoji, no stage directions.'
)

# Normalized phrases that end the conversation session.
END_PHRASES = (
    'goodbye',
    'good bye',
    'bye',
    'bye bye',
    'end chat',
    'stop chatting',
    'stop talking',
    "that's all",
    'thats all',
)

# Sentence terminator followed by whitespace. Requiring the whitespace keeps
# decimals ("3.5 volts") and mid-token dots intact; a terminator at the very
# end of the stream is handled by flush().
_SENTENCE_END = re.compile(r'([.!?…]+)(?=\s)')

# Characters the model may emit that TTS would read aloud or garble.
_MARKDOWN_CHARS = re.compile(r'[*_`#>|~\[\]]')
_WS = re.compile(r'\s+')
_NORM_STRIP = re.compile(r'[^a-z\' ]+')


def clean_for_tts(text):
    """Strip markdown-ish characters and collapse whitespace."""
    return _WS.sub(' ', _MARKDOWN_CHARS.sub('', text)).strip()


def _normalize(text):
    return _WS.sub(' ', _NORM_STRIP.sub(' ', text.lower())).strip()


def is_end_phrase(text):
    """True if the utterance asks to end the conversation."""
    n = _normalize(text)
    return any(n == p or n.endswith(' ' + p) for p in END_PHRASES)


class SentenceChunker:
    """Accumulates streamed LLM tokens and yields complete sentences, so the
    mouth can start speaking sentence one while the model writes sentence
    two. If the model rambles without punctuation, the buffer is force-split
    at a word boundary to bound speech latency."""

    def __init__(self, max_buffer_chars=240):
        self.max_buffer_chars = int(max_buffer_chars)
        self._buf = ''

    def feed(self, token):
        """Add a streamed token; return the list of sentences now complete."""
        self._buf += token
        out = []
        while True:
            m = _SENTENCE_END.search(self._buf)
            if m:
                end = m.end(1)
                sentence = clean_for_tts(self._buf[:end])
                self._buf = self._buf[end:]
                if sentence:
                    out.append(sentence)
                continue
            if len(self._buf) >= self.max_buffer_chars:
                cut = self._buf.rfind(' ', 0, self.max_buffer_chars)
                if cut <= 0:
                    cut = self.max_buffer_chars
                piece = clean_for_tts(self._buf[:cut])
                self._buf = self._buf[cut:]
                if piece:
                    out.append(piece)
                continue
            return out

    def flush(self):
        """Return whatever remains (end of stream), emptying the buffer."""
        piece = clean_for_tts(self._buf)
        self._buf = ''
        return piece


class ChatHistory:
    """System prompt plus a rolling window of the last max_turns exchanges,
    so a long conversation can't grow the prompt (and the Jetson's RAM/
    latency) without bound."""

    def __init__(self, system_prompt, max_turns=12):
        self.system_prompt = system_prompt
        self.max_turns = int(max_turns)
        self._turns = []  # flat list of {'role': ..., 'content': ...}

    def add_user(self, text):
        self._turns.append({'role': 'user', 'content': text})
        self._trim()

    def add_assistant(self, text):
        if text:
            self._turns.append({'role': 'assistant', 'content': text})
            self._trim()

    def clear(self):
        self._turns = []

    def messages(self):
        """Full message list for the Ollama /api/chat request."""
        return ([{'role': 'system', 'content': self.system_prompt}]
                + list(self._turns))

    def _trim(self):
        keep = self.max_turns * 2  # a turn = user + assistant
        if len(self._turns) > keep:
            del self._turns[:len(self._turns) - keep]
