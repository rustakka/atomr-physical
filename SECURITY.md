# Security policy

## Reporting vulnerabilities

Report security issues privately to the maintainers via GitHub's
security-advisory flow:
<https://github.com/rustakka/atomr-physical/security/advisories/new>.

Please **do not file a public issue** for unpatched security problems.

## Scope

atomr-physical sits between application code and hardware drivers /
ROS2. The security-relevant code paths are:

- **The `SafetyEnvelope`** (`atomr-physical-actuation`): the clamp /
  reject boundary on actuation setpoints. A bug here can let an
  out-of-range setpoint reach a real actuator — treat envelope-bypass
  bugs as high severity.
- **The device-contract traits** (`atomr-physical-core`): the
  `Sensor` / `Actuator` seam. Drivers are caller-supplied; the trait
  boundary must not let a misbehaving driver corrupt the actor above
  it.
- **The ROS2 bridge** (`atomr-physical-ros2`): topic / message-type
  mapping, and (with the `rclrs` feature) the live transport. Crafted
  ROS2 messages are untrusted input.
- **Python bindings** (`atomr-physical-py-bindings`): GIL containment
  and FFI safety across the `atomr_physical._native` boundary.

## What we treat as a security issue

- Memory unsafety in any `unsafe` block.
- A `SafetyEnvelope` that admits a setpoint outside its declared
  bounds, or a path that bypasses the envelope entirely.
- DoS via crafted ROS2 messages or sensor-driver inputs.
- Path traversal or injection in device-descriptor / topic-name
  handling.
- Information disclosure in error messages.
- Cross-FFI memory-safety violations in the Python bindings.

## What we do not treat as a security issue

- The physical behaviour of a caller-supplied driver — atomr-physical
  forwards commands to a driver that may itself be unsafe; auditing the
  driver is the integrator's responsibility.
- A `SafetyEnvelope` configured with bounds that are themselves unsafe
  for the hardware — choosing correct bounds is the integrator's
  responsibility.
- ROS2 transport-level security (DDS authentication / encryption) —
  that is configured in the ROS2 layer, not here.
