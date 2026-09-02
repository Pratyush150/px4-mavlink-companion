"""Path setup shared by the examples.

Lets you run the examples straight from a checkout without installing anything::

    python3 examples/heartbeat_check.py --sim
"""

from __future__ import annotations

import os
import sys


def add_src_to_path() -> None:
    """Put ``<repo>/src`` on ``sys.path`` if mavbridge is not installed."""
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "src")
    if os.path.isdir(src) and src not in sys.path:
        sys.path.insert(0, src)
