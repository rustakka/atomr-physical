"""Facade over :mod:`atomr_physical._native.errors`.

Re-exports the exception hierarchy::

    PhysicalError
     ├─ OutOfRange
     └─ DeviceFault
"""

from ._native import errors as _sub

globals().update({k: getattr(_sub, k) for k in dir(_sub) if not k.startswith("_")})
__all__ = [k for k in dir(_sub) if not k.startswith("_")]
