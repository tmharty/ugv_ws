"""Tiny conversational state: at most one pending confirmation.

go_to_point requires a verbal "yes" before the robot navigates. This holds the
validated intent until the child confirms, denies, says something else
(implicit cancel), or the window expires. Pure Python, injectable clock.
"""

import time


class DialogContext:
    def __init__(self, confirm_timeout_s=15.0, clock=None):
        self.confirm_timeout_s = confirm_timeout_s
        self._clock = clock or time.monotonic
        self._pending = None
        self._deadline = 0.0

    def request_confirmation(self, validated_intent):
        self._pending = validated_intent
        self._deadline = self._clock() + self.confirm_timeout_s

    def pending(self):
        """The intent awaiting confirmation, or None (expiry included)."""
        if self._pending is not None and self._clock() > self._deadline:
            self._pending = None
        return self._pending

    def resolve(self, affirmed):
        """Consume the pending intent. Returns it if affirmed, else None."""
        pending = self.pending()
        self._pending = None
        return pending if affirmed else None

    def cancel(self):
        self._pending = None
