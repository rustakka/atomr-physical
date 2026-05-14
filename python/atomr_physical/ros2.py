"""Facade over :mod:`atomr_physical._native.ros2`.

Re-exports ``Ros2Endpoint`` and ``TopicMap`` — the offline topic-graph
plan the ROS2 bridge spins up. Driving a live graph needs the native
extension built with the ``rclrs`` feature; see ``docs/ros2-bridge.md``.
"""

from ._native import ros2 as _sub

globals().update({k: getattr(_sub, k) for k in dir(_sub) if not k.startswith("_")})
__all__ = [k for k in dir(_sub) if not k.startswith("_")]
