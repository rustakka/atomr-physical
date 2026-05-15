# Python API

The Python API is a **first-class overlay**, not a wrapper bolted on
afterward. The same Rust value types back both languages: a Python
`Quantity` *is* an `atomr_physical_core::Quantity` behind a PyO3
`#[pyclass]` — there is no second implementation to drift.

## The three layers

atomr-physical's Python support follows the same three-layer pattern
[atomr](https://github.com/rustakka/atomr) and
[atomr-agents](https://github.com/rustakka/atomr-agents) use:

1. **Native Rust crates** (`atomr-physical-core`, `-sensing`,
   `-actuation`, `-robotics`, `-ros2`) — pure Rust, no Python
   dependency.
2. **The PyO3 binding crate** (`atomr-physical-py-bindings`) — depends
   on every native crate, wraps their types as Python classes, and
   compiles to a `cdylib` named `_native`. Built with
   [maturin](https://www.maturin.rs/).
3. **The pure-Python facade** (`python/atomr_physical/`) — thin `.py`
   modules that re-export the native submodules, plus the top-level
   convenience names, the `py.typed` marker, and (over time) any
   pure-Python helpers.

## Module map

| Python module | Native submodule | Exposes |
|---|---|---|
| `atomr_physical` | — | top-level convenience re-exports + submodule facades |
| `atomr_physical.errors` | `_native.errors` | `PhysicalError`, `OutOfRange`, `DeviceFault` |
| `atomr_physical.core` | `_native.core` | `Quantity`, `Reading`, `Command`, `CommandAck`, `DeviceDescriptor` |
| `atomr_physical.sensing` | `_native.sensing` | `SamplingPolicy`, `Calibration` |
| `atomr_physical.actuation` | `_native.actuation` | `SafetyEnvelope` |
| `atomr_physical.robotics` | `_native.robotics` | `Joint`, `RobotModel` |
| `atomr_physical.ros2` | `_native.ros2` | the offline ROS2 plan: `Ros2Endpoint`, `TopicMap`, `Ros2Plan`, `Ros2ServiceEndpoint`, `Ros2ActionEndpoint`, `Ros2ParamDecl`, `QosProfile`, `Ros2ClockSource`, and a read-only `CodecRegistry` view |

The top-level package re-exports the most-used classes, so
`from atomr_physical import Quantity, RobotModel, SafetyEnvelope` works
directly.

## Building the extension

```bash
pip install maturin
maturin develop -m crates/py-bindings/Cargo.toml
pip install -e ".[dev]"
pytest python/atomr_physical/tests
```

`maturin develop` compiles the `cdylib` into the active venv and
installs the Python facade from `python/atomr_physical/`. The maturin
config lives in `pyproject.toml` (`module-name =
"atomr_physical._native"`, `python-source = "python"`).

## Crossing the FFI boundary

Units and control modes cross as short strings rather than enum
objects — `"rad"`, `"m/s"`, `"position"`, `"duty"`:

```python
from atomr_physical import Quantity, Command

q = Quantity(1.57, "rad")
print(q.value, q.unit)            # 1.57 rad

cmd = Command("joint-0", setpoint=0.8, mode="position", unit="rad")
print(cmd.actuator, cmd.mode)     # joint-0 position
```

Errors map onto the exception hierarchy — a Rust
`PhysicalError::OutOfRange` becomes a Python `OutOfRange`:

```python
from atomr_physical import SafetyEnvelope
from atomr_physical.errors import OutOfRange

env = SafetyEnvelope.rejecting(0.0, 1.0)
try:
    env.enforce("a1", 5.0)
except OutOfRange as e:
    print("rejected:", e)
```

## What's exposed at 0.1.0

The Python overlay covers the **value types and the device contract** —
quantities, readings, commands, descriptors, the safety / calibration
policies, the kinematic model, and the ROS2 topic plan. These are the
types a Python caller constructs, inspects, and asserts on.

The **live actor surface** (driving a `SensorActor` sampling loop or an
`ActuatorActor` queue from Python, async coroutines over
`pyo3-async-runtimes`) lands alongside the Phase-2 actor wiring
described in [architecture.md](architecture.md). The facade layout is
already shaped for it — new native classes slot into the existing
submodules and their `.py` facades pick them up automatically.

## Adding a binding

1. Add the `#[pyclass]` to the matching `crates/py-bindings/src/*.rs`
   submodule and register it in that file's `register` fn.
2. If it is a commonly-used type, add a top-level re-export in
   `python/atomr_physical/__init__.py`'s `if _native is not None:`
   block and to `__all__`.
3. The per-submodule `.py` facade
   (`globals().update(... dir(_sub) ...)`) picks it up with no change.
4. Add a smoke-test assertion in
   `python/atomr_physical/tests/test_smoke.py`.
