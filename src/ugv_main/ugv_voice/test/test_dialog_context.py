"""Confirmation-window state machine, with an injected clock."""

from ugv_voice.dialog_context import DialogContext


def make(timeout=15.0):
    clock = {'t': 0.0}
    ctx = DialogContext(confirm_timeout_s=timeout, clock=lambda: clock['t'])
    return ctx, clock


def test_confirm_flow():
    ctx, _ = make()
    ctx.request_confirmation('nav-intent')
    assert ctx.pending() == 'nav-intent'
    assert ctx.resolve(True) == 'nav-intent'
    assert ctx.pending() is None  # consumed


def test_deny_flow():
    ctx, _ = make()
    ctx.request_confirmation('nav-intent')
    assert ctx.resolve(False) is None
    assert ctx.pending() is None


def test_expiry():
    ctx, clock = make(timeout=15.0)
    ctx.request_confirmation('nav-intent')
    clock['t'] = 15.1
    assert ctx.pending() is None
    assert ctx.resolve(True) is None  # a late "yes" must not navigate


def test_cancel():
    ctx, _ = make()
    ctx.request_confirmation('nav-intent')
    ctx.cancel()
    assert ctx.pending() is None
