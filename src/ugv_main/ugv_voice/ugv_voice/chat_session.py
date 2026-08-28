"""Pure conversation logic for the chat node — zero ROS imports.

Same pattern as intent_schema.py: everything unit-testable without a ROS
runtime lives here; chat_node.py is the thin ROS wrapper around it.

Chat mode speaks LLM-generated text aloud (speech safety is the persona
prompt, not a script) and lets the model *propose* tool calls. The
ToolLoop below runs the bounded agent loop; every proposed call is handed
to a dispatch callback that — in chat_node — runs it through tools.py's
validator. Nothing in this module can move the robot.
"""

import re

from .tools import parse_tool_calls, schemas as tool_schemas

# Spoken output: short, plain, honest-machine. The tools are described
# honestly, and the model is told to announce a motion before calling it —
# that sentence is the speak-then-act reaction window.
DEFAULT_SYSTEM_PROMPT = (
    'You are the voice of a small tracked robot called Dark Gooder or Gooder for short. '
    'You are a good robot who tries to do what is best. '
    'You do not have emotions but try to be a helpful robot. '
    'You try to answer questions with short responces, but you also make jokes. '
    'You mostly talk to young children so you try to put things simply and are never crude. '
    'You can drive short, slow distances, turn, spin, go to saved points, '
    'blink your lights and read your battery, using your tools. When asked '
    'to move, say in one short sentence what you are about to do, then call '
    'the tool. Do one action per request. Never pretend to move without '
    'calling a tool, and never claim to have feelings or to be human. '
    'Your replies are spoken aloud through text-to-speech: answer in one to '
    'three short sentences of plain prose. No lists, no markdown, no emoji, '
    'no stage directions.'
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

    def add_assistant(self, text, tool_calls=None):
        """Record the model's reply. tool_calls is the raw Ollama list (kept
        verbatim so the tool results that follow line up with it)."""
        if not text and not tool_calls:
            return
        msg = {'role': 'assistant', 'content': text or ''}
        if tool_calls:
            msg['tool_calls'] = list(tool_calls)
        self._turns.append(msg)
        self._trim()

    def add_tool_result(self, tool_name, content):
        self._turns.append({'role': 'tool', 'tool_name': tool_name,
                            'content': content})
        self._trim()

    def clear(self):
        self._turns = []

    def messages(self):
        """Full message list for the Ollama /api/chat request."""
        return ([{'role': 'system', 'content': self.system_prompt}]
                + list(self._turns))

    def _trim(self):
        # A turn starts at a user message and includes whatever assistant /
        # tool messages follow it; keep the last max_turns of them so the
        # window never starts on a dangling tool result.
        user_idx = [i for i, m in enumerate(self._turns) if m['role'] == 'user']
        if len(user_idx) > self.max_turns:
            del self._turns[:user_idx[-self.max_turns]]


class TurnFailed(Exception):
    """The LLM transport failed. spoke_any tells the caller whether the
    user already heard part of a reply (so a fallback must not re-act)."""

    def __init__(self, cause, spoke_any):
        super().__init__(str(cause))
        self.cause = cause
        self.spoke_any = spoke_any


class ToolLoop:
    """Bounded agent loop over an injected streaming LLM.

    run() streams one model reply, speaks prose sentence-by-sentence via
    ``speak``, hands every proposed tool call to ``run_tool`` and feeds the
    returned result text back to the model as a tool-role message so it
    can narrate the outcome. At most ``max_tool_rounds`` rounds may carry
    tool calls; the final round is requested without tools so a confused
    model has to answer in prose and cannot loop.

    Callbacks (all supplied by chat_node, none imported here):
      stream_fn(messages, tools)  -> iterable of Ollama chunk dicts;
                                     tools is a schema list or None
      speak(sentence)             -> None
      run_tool(ToolCall, spoke)   -> result text for the model; ``spoke``
                                     is True if prose was spoken this round
      still_current()             -> False once a barge-in invalidates the turn
    """

    def __init__(self, history, max_tool_rounds=3, tools_enabled=True):
        self.history = history
        self.max_tool_rounds = int(max_tool_rounds)
        self.tools_enabled = bool(tools_enabled)

    def run(self, user_text, stream_fn, speak, run_tool, still_current=lambda: True):
        self.history.add_user(user_text)
        spoken = []
        tool_calls_seen = 0
        for round_no in range(self.max_tool_rounds + 1):
            tools = (tool_schemas()
                     if self.tools_enabled and round_no < self.max_tool_rounds
                     else None)
            chunker = SentenceChunker()
            content, raw_calls, spoke_this_round = [], [], False
            try:
                for chunk in stream_fn(self.history.messages(), tools):
                    if not still_current():
                        return spoken
                    message = chunk.get('message') or {}
                    token = message.get('content') or ''
                    content.append(token)
                    for sentence in chunker.feed(token):
                        speak(sentence)
                        spoken.append(sentence)
                        spoke_this_round = True
                    calls = message.get('tool_calls')
                    if isinstance(calls, list):
                        raw_calls.extend(calls)
                    if chunk.get('done'):
                        break
                tail = chunker.flush()
            except Exception as e:  # transport / JSON failure
                raise TurnFailed(e, spoke_any=bool(spoken))
            if tail and still_current():
                speak(tail)
                spoken.append(tail)
                spoke_this_round = True
            if not still_current():
                return spoken
            self.history.add_assistant(clean_for_tts(''.join(content)), raw_calls)
            if not raw_calls:
                return spoken
            for call in parse_tool_calls({'tool_calls': raw_calls}):
                tool_calls_seen += 1
                result = run_tool(call, spoke_this_round)
                self.history.add_tool_result(call.name, str(result))
                if not still_current():
                    return spoken
        return spoken
