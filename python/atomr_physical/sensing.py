"""Facade over :mod:`atomr_physical._native.sensing`.

Re-exports ``SamplingPolicy`` and ``Calibration`` — the policy and
correction types a ``SensorActor`` is built from.
"""

from ._native import sensing as _sub

globals().update({k: getattr(_sub, k) for k in dir(_sub) if not k.startswith("_")})
__all__ = [k for k in dir(_sub) if not k.startswith("_")]
