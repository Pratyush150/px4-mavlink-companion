"""Test configuration.

Adds ``src/`` to ``sys.path`` so the suite runs from a checkout with nothing
installed, and provides a deterministic fake clock.

Every test in this suite passes with ``pymavlink`` absent. That is deliberate:
the parts of mavbridge you most need to trust (stale-telemetry detection, mode
decoding, bandwidth budgeting) must be testable on a laptop with no flight
stack installed.
"""

from __future__ import annotations

import os
import sys

import pytest

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


class FakeClock:
    """A monotonic clock you control.

    ``clock()`` returns the current value; ``clock.sleep(dt)`` advances it.
    Passing ``clock`` and ``clock.sleep`` into the library makes time-dependent
    behaviour deterministic and instant.
    """

    def __init__(self, start: float = 0.0) -> None:
        self.now = float(start)

    def __call__(self) -> float:
        return self.now

    def sleep(self, dt: float) -> None:
        """Advance the clock by *dt* seconds (minimum one microsecond)."""
        self.now += max(float(dt), 1e-6)

    def advance(self, dt: float) -> float:
        """Advance the clock and return the new value."""
        self.now += float(dt)
        return self.now


@pytest.fixture
def clock() -> FakeClock:
    """A fresh :class:`FakeClock` starting at zero."""
    return FakeClock()
