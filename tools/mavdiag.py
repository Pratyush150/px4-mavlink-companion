#!/usr/bin/env python3
"""Standalone launcher for the link diagnostic.

Identical to ``python -m mavbridge.diagnose``, but runnable straight from a
checkout with nothing installed::

    python3 tools/mavdiag.py --port auto
    python3 tools/mavdiag.py --conn udp:0.0.0.0:14540 --duration 15 --json
    python3 tools/mavdiag.py --sim px4 --fault frozen        # no hardware needed

Exit codes: 0 healthy, 1 problems found, 2 could not connect.
"""

from __future__ import annotations

import os
import sys

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "src")
if os.path.isdir(_SRC) and _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from mavbridge.diagnose import main  # noqa: E402  (path setup must come first)

if __name__ == "__main__":
    sys.exit(main())
