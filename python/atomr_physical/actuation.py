"""Facade over :mod:`atomr_physical._native.actuation`.

Re-exports ``SafetyEnvelope`` — the min/max clamp an ``ActuatorActor``
enforces before a command reaches hardware.
"""

from ._native import actuation as _sub

globals().update({k: getattr(_sub, k) for k in dir(_sub) if not k.startswith("_")})
__all__ = [k for k in dir(_sub) if not k.startswith("_")]
