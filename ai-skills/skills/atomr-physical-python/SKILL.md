---
name: atomr-physical-python
description: Use when working with the atomr-physical Python overlay — `pip install atomr-physical`, importing from `atomr_physical.*`, building the native extension with maturin, or adding a new PyO3 binding. Triggers on `import atomr_physical`, `from atomr_physical import`, `maturin develop`, or editing `crates/py-bindings/`.
---

# atomr-physical Python overlay

The Python API is a **first-class overlay** — the same Rust value types
back both languages. A Python `Quantity` *is* an
`atomr_physical_core::Quantity` behind a PyO3 `#[pyclass]`.

## The mental model

- **Three layers.** Native Rust crates → the `atomr-physical-py-bindings`
  PyO3 `cdylib` (`atomr_physical._native`) → thin pure-Python facades
  in `python/atomr_physical/`.
- **One native submodule per Rust crate.** `_native.errors`,
  `_native.core`, `_native.sensing`, `_native.actuation`,
  `_native.robotics`, `_native.ros2` — each mirrored by a `.py` facade.
- **Units cross as strings.** `"rad"`, `"m/s"`, `"position"`,
  `"duty"` — not enum objects.
- **Errors map onto the exception hierarchy.** A Rust
  `PhysicalError::OutOfRange` becomes a Python `OutOfRange`.

## Install / build

```bash
# Published wheel:
pip install atomr-physical

# Editable, against the local checkout:
pip install maturin
maturin develop -m crates/py-bindings/Cargo.toml
pip install -e ".[dev]"
pytest python/atomr_physical/tests
```

## Using it

```python
from atomr_physical import Quantity, Command, SafetyEnvelope, RobotModel, Joint
from atomr_physical.errors import OutOfRange

q = Quantity(1.57, "rad")
print(q.value, q.unit)                       # 1.57 rad

cmd = Command("joint-0", setpoint=0.8, mode="position", unit="rad")

env = SafetyEnvelope.rejecting(0.0, 1.0)
try:
    env.enforce("joint-0", 5.0)
except OutOfRange as e:
    print("rejected:", e)

model = RobotModel()
model.add_joint(Joint("j1", "shoulder_pan", actuator="a1", feedback="s1"))
```

## Adding a binding

1. Add the `#[pyclass]` to the matching `crates/py-bindings/src/*.rs`
   submodule and register it in that file's `register` fn.
2. For a commonly-used type, add a top-level re-export in
   `python/atomr_physical/__init__.py` (and to `__all__`).
3. The per-submodule `.py` facade picks it up automatically.
4. Add a smoke-test assertion in
   `python/atomr_physical/tests/test_smoke.py`.

## Canonical references

- [`docs/python-api.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/python-api.md) — the three-layer pattern + the full module map
- `crates/py-bindings/src/` — the PyO3 submodules
- `python/atomr_physical/` — the Python facades
- `pyproject.toml` — the maturin config

## Common mistakes

- **`import atomr_physical` before `maturin develop`.** The facade
  imports `_native`; without the built extension the submodule
  attributes are absent. Build the extension first.
- **Passing a `Unit` object.** Units are strings on the FFI boundary —
  `Quantity(1.0, "rad")`, not an enum.
- **Expecting the live actor surface.** At 0.1.0 the overlay exposes
  the value types and the device contract; driving a `SensorActor`
  loop from Python lands with the Phase-2 actor wiring.
