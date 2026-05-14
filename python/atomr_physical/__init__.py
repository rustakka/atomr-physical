"""atomr-physical — physical sensing, output, and ROS2-integrated robotics on atomr actors.

The native PyO3 extension lives in :mod:`atomr_physical._native` and is
split into per-domain submodules — ``errors``, ``core``, ``sensing``,
``actuation``, ``robotics``, and ``ros2`` — mirroring the upstream
``atomr`` / ``atomr-agents`` Python binding layout. Every submodule has
a one-to-one ``.py`` facade under ``atomr_physical/`` so pure-Python
helpers and type stubs can grow alongside the native types.

Build the native extension::

    pip install maturin
    maturin develop -m crates/py-bindings/Cargo.toml

Common imports (top-level convenience names)::

    from atomr_physical import (
        Quantity, Reading, Command, CommandAck, DeviceDescriptor,
        SamplingPolicy, Calibration,
        SafetyEnvelope,
        Joint, RobotModel,
        Ros2Endpoint, TopicMap,
    )
    from atomr_physical.errors import PhysicalError, OutOfRange, DeviceFault

Plan a robot and its ROS2 graph::

    from atomr_physical import Joint, RobotModel, Ros2Endpoint, TopicMap

    model = RobotModel()
    model.add_joint(Joint("j1", "shoulder_pan", actuator="a1", feedback="s1"))

    topics = TopicMap()
    topics.bind_sensor("s1", Ros2Endpoint.publish("/robot/joint_states",
                                                  "sensor_msgs/msg/JointState"))
"""

from importlib import metadata as _metadata

try:
    from . import _native
except ImportError as _e:  # pragma: no cover - native extension not built yet
    _native = None
    _import_err = _e
else:
    _import_err = None

if _native is not None:
    # ----- subpackages re-exported as attributes ------------------------
    errors = _native.errors
    core = _native.core
    sensing = _native.sensing
    actuation = _native.actuation
    robotics = _native.robotics
    ros2 = _native.ros2

    # ----- top-level convenience re-exports -----------------------------
    Quantity = core.Quantity
    Reading = core.Reading
    Command = core.Command
    CommandAck = core.CommandAck
    DeviceDescriptor = core.DeviceDescriptor

    SamplingPolicy = sensing.SamplingPolicy
    Calibration = sensing.Calibration

    SafetyEnvelope = actuation.SafetyEnvelope

    Joint = robotics.Joint
    RobotModel = robotics.RobotModel

    Ros2Endpoint = ros2.Ros2Endpoint
    TopicMap = ros2.TopicMap

try:
    __version__ = _metadata.version("atomr-physical")
except _metadata.PackageNotFoundError:  # editable installs / running from source
    __version__ = "0.0.0+unknown"

__all__ = [
    # submodule facades
    "errors",
    "core",
    "sensing",
    "actuation",
    "robotics",
    "ros2",
    # core value types
    "Quantity",
    "Reading",
    "Command",
    "CommandAck",
    "DeviceDescriptor",
    # sensing
    "SamplingPolicy",
    "Calibration",
    # actuation
    "SafetyEnvelope",
    # robotics
    "Joint",
    "RobotModel",
    # ros2
    "Ros2Endpoint",
    "TopicMap",
]
