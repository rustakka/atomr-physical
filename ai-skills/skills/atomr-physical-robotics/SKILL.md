---
name: atomr-physical-robotics
description: Use when orchestrating a robot with atomr-physical — building a RobotModel of Joints, registering SensorActors and ActuatorActors under a RobotActor supervisor, or describing a robot's kinematic structure. Triggers on `RobotActor::new`, `RobotModel::`, `Joint::new`, or `.add_sensor` / `.add_actuator`.
---

# atomr-physical robotics

Robot-level orchestration: a `RobotActor` is the supervisor at the top
of a physical system, owning a kinematic `RobotModel` and a tree of
sensor / actuator actors.

## The mental model

- **A `RobotModel` is the kinematic description.** A `Vec<Joint>` plus
  `auxiliary_sensors` (sensors not bound to a joint — chassis IMU,
  battery monitor).
- **A `Joint` pairs an actuator with optional feedback.** `Joint { id,
  name, actuator, feedback }` — `feedback` is the `SensorId` reporting
  that joint's state, if instrumented.
- **A `RobotActor` is the supervisor.** It owns the model plus child
  `SensorActor`s / `ActuatorActor`s keyed by id. A driver fault
  restarts one subtree, not the process (Phase 2 wires the supervision;
  see `docs/architecture.md`).

## Building a robot

```rust
use atomr_physical::prelude::*;
use atomr_physical::robotics::{Joint, RobotActor, RobotModel};
use atomr_physical::sensing::{SamplingPolicy, SensorActor};
use atomr_physical::actuation::{ActuatorActor, SafetyEnvelope};

// 1. Describe the kinematics.
let model = RobotModel::new()
    .with_joint(
        Joint::new(JointId::from("j1"), "shoulder_pan", ActuatorId::from("a1"))
            .with_feedback(SensorId::from("s1")),
    )
    .with_joint(Joint::new(JointId::from("j2"), "shoulder_lift", ActuatorId::from("a2")))
    .with_auxiliary_sensor(SensorId::from("imu0"));

// 2. Build the supervisor and register the device actors.
let mut robot = RobotActor::new(RobotId::from("arm-1"), model);
robot.add_sensor(SensorActor::new(s1_driver, SamplingPolicy::default_rate()));
robot.add_actuator(
    ActuatorActor::new(a1_driver).with_envelope(SafetyEnvelope::clamping(-1.57, 1.57)),
);

assert_eq!(robot.child_count(), 2);
let joint = robot.model().joint(&JointId::from("j1")).unwrap();
```

## Canonical references

- [`docs/architecture.md`](https://github.com/rustakka/atomr-physical/blob/main/docs/architecture.md) — the device-actor model + the Phase-2 supervision roadmap
- `crates/robotics/src/lib.rs` — `RobotActor`, `RobotModel`, `Joint`
- the `atomr-physical-sensing` / `atomr-physical-actuation` skills — building the child actors

## Common mistakes

- **Adding an actor whose id isn't in the model.** `add_sensor` /
  `add_actuator` key by the actor's id; keep those ids consistent with
  the `Joint.actuator` / `Joint.feedback` references in the model.
- **Expecting live supervision at 0.1.0.** `RobotActor` owns the child
  actors as a plain map today; the supervised restart tree is Phase 2.
- **Modelling a joint with no actuator.** Every `Joint` is built around
  the actuator that drives it — `feedback` is the optional half.
