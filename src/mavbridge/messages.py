"""Minimal, dependency-free access layer over "a MAVLink message".

Three different things end up flowing through this library:

* real ``pymavlink`` message objects (``msg.get_type()``, attribute access),
* plain dicts (from a JSON log replay, a test fixture, or a router),
* :class:`SimpleMessage` instances produced by :mod:`mavbridge.simulator`.

Rather than special-casing those everywhere, every consumer in this package
goes through :func:`message_type` and :func:`field`. That is also what makes it
possible to unit-test the watchdog and telemetry normalisation with no
``pymavlink`` installed.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, Mapping, Optional

__all__ = ["SimpleMessage", "message_type", "field", "fields_present", "to_dict"]


class SimpleMessage:
    """A duck-typed stand-in for a ``pymavlink`` message.

    Exposes the same two things consumers actually use: ``get_type()`` and
    attribute access for payload fields.

    Args:
        name: MAVLink message name, e.g. ``"ATTITUDE"``. Positional-only.
        **fields: Payload fields.

    Example:
        >>> m = SimpleMessage("ATTITUDE", roll=0.1, pitch=0.0, yaw=1.57)
        >>> m.get_type()
        'ATTITUDE'
        >>> round(m.roll, 2)
        0.1
    """

    __slots__ = ("_type", "_fields")

    def __init__(self, name: str, /, **fields: Any) -> None:
        # `name` is positional-only on purpose: HEARTBEAT has a payload field
        # literally called `type`, and it must be passable as a keyword.
        self._type = name
        self._fields: Dict[str, Any] = dict(fields)

    def get_type(self) -> str:
        """Return the MAVLink message name."""
        return self._type

    def to_dict(self) -> Dict[str, Any]:
        """Return a copy of the payload fields plus a ``mavpackettype`` key."""
        out = dict(self._fields)
        out["mavpackettype"] = self._type
        return out

    def __getattr__(self, name: str) -> Any:
        try:
            return self._fields[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __contains__(self, name: str) -> bool:
        return name in self._fields

    def __repr__(self) -> str:
        inner = ", ".join(f"{k}={v!r}" for k, v in self._fields.items())
        return f"SimpleMessage({self._type!r}, {inner})"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SimpleMessage):
            return NotImplemented
        return self._type == other._type and self._fields == other._fields


def message_type(msg: Any) -> str:
    """Return the MAVLink message name for any supported message representation.

    Args:
        msg: A ``pymavlink`` message, a :class:`SimpleMessage`, or a mapping
            containing ``mavpackettype`` / ``type``.

    Returns:
        The uppercase message name, or ``"UNKNOWN"`` if it cannot be determined.
    """
    getter = getattr(msg, "get_type", None)
    if callable(getter):
        try:
            return str(getter())
        except Exception:  # pragma: no cover - malformed third-party object
            return "UNKNOWN"
    if isinstance(msg, Mapping):
        for key in ("mavpackettype", "type", "name"):
            if key in msg:
                return str(msg[key])
    return "UNKNOWN"


def field(msg: Any, name: str, default: Any = None) -> Any:
    """Read one payload field from any supported message representation.

    Args:
        msg: Message object or mapping.
        name: Field name.
        default: Returned when the field is absent or unreadable.

    Returns:
        The field value, or ``default``.
    """
    if isinstance(msg, Mapping):
        return msg.get(name, default)
    try:
        return getattr(msg, name)
    except AttributeError:
        return default


def fields_present(msg: Any, names: Iterable[str]) -> bool:
    """Return ``True`` if every name in *names* is readable on *msg*."""
    sentinel = object()
    return all(field(msg, n, sentinel) is not sentinel for n in names)


def to_dict(msg: Any) -> Dict[str, Any]:
    """Best-effort conversion of a message to a plain dict of payload fields."""
    if isinstance(msg, Mapping):
        return dict(msg)
    converter = getattr(msg, "to_dict", None)
    if callable(converter):
        try:
            return dict(converter())
        except Exception:  # pragma: no cover - malformed third-party object
            pass
    out: Dict[str, Any] = {}
    for name in getattr(msg, "_fieldnames", ()) or ():
        value: Optional[Any] = field(msg, name)
        out[name] = value
    out.setdefault("mavpackettype", message_type(msg))
    return out
