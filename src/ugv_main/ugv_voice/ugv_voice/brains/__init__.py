"""Brain backends: transcript text -> raw intent dict.

Every backend returns the same shape — {"intent": name, "params": {...}} —
and every backend's output goes through the identical intent_schema validator.
Backends never produce spoken text.
"""


class Brain:
    """Interface. parse() must never raise and must return a dict."""

    name = 'base'

    def parse(self, text):
        raise NotImplementedError


def make_brain(backend, logger=None):
    """Factory keyed by the brain_backend ROS param. Unknown or not-yet-built
    backends fall back to rules — the robot must never come up brainless."""
    from .rule_brain import RuleBrain
    if backend == 'rules':
        return RuleBrain()
    if logger is not None:
        logger.warn(f"brain_backend '{backend}' not available yet — "
                    "falling back to 'rules'")
    return RuleBrain()
