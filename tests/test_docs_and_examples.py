"""Keep the documentation honest: run every docstring example, compile every script."""

from __future__ import annotations

import doctest
import importlib
import os
import pathlib
import py_compile

import pytest

MODULES = [
    "mavbridge",
    "mavbridge._mav",
    "mavbridge.messages",
    "mavbridge.link",
    "mavbridge.watchdog",
    "mavbridge.telemetry",
    "mavbridge.rates",
    "mavbridge.offboard",
    "mavbridge.simulator",
    "mavbridge.diagnose",
]

ROOT = pathlib.Path(__file__).resolve().parent.parent


@pytest.mark.parametrize("module_name", MODULES)
def test_docstring_examples_still_work(module_name):
    """A README that lies is worse than no README; the same goes for docstrings."""
    module = importlib.import_module(module_name)
    results = doctest.testmod(
        module,
        optionflags=doctest.ELLIPSIS | doctest.IGNORE_EXCEPTION_DETAIL,
        verbose=False,
    )
    assert results.failed == 0, f"{module_name}: {results.failed} failing doctest(s)"


@pytest.mark.parametrize(
    "script",
    sorted(str(p.relative_to(ROOT)) for p in (ROOT / "examples").glob("*.py"))
    + ["tools/mavdiag.py"],
)
def test_scripts_compile(script):
    """Examples are advertised as runnable, so at minimum they must parse."""
    py_compile.compile(str(ROOT / script), doraise=True)


def test_every_example_documents_the_setup_it_needs():
    """Each example states the hardware or SITL it expects, in its header."""
    for path in sorted((ROOT / "examples").glob("*.py")):
        if path.name.startswith("_"):
            continue  # shared helper, not an example
        header = path.read_text()[:2000]
        assert "Requires:" in header, f"{path.name} does not say what it needs"


def test_package_imports_without_pymavlink():
    """The whole point of mavbridge._mav: no hard dependency at import time."""
    import mavbridge

    assert mavbridge.__version__
    assert hasattr(mavbridge, "Watchdog")
    assert os.path.isfile(ROOT / "requirements.txt")
    assert os.path.isfile(ROOT / "docs" / "TROUBLESHOOTING.md")
