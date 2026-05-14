"""Facade over :mod:`atomr_physical._native.robotics`.

Re-exports ``Joint`` and ``RobotModel`` — the kinematic description a
``RobotActor`` supervises.
"""

from ._native import robotics as _sub

globals().update({k: getattr(_sub, k) for k in dir(_sub) if not k.startswith("_")})
__all__ = [k for k in dir(_sub) if not k.startswith("_")]
