# ai-skills/

Skills for AI coding assistants working on **projects that depend on
atomr-physical** — not for editing atomr-physical itself. They follow
the standard `SKILL.md` + frontmatter convention used by Claude Code,
Claude Agent SDK, and other agentic tools.

These skills are deliberately separate from the repo's own dev tooling
so distributing them to consumers does not entangle atomr-physical's
internal development workflow.

## What's here

| Skill | Use when… |
|---|---|
| `atomr-physical-quickstart` | Standing up the first project — picking feature flags, wrapping a driver in a `SensorActor` / `ActuatorActor`, running against `MockSensor` / `MockActuator` |
| `atomr-physical-sensing` | Reading from sensors — implementing the `Sensor` contract trait, picking a `SamplingPolicy`, applying a `Calibration` |
| `atomr-physical-actuation` | Driving actuators — implementing the `Actuator` contract trait, configuring a `SafetyEnvelope` (clamp vs reject) |
| `atomr-physical-robotics` | Orchestrating a robot — building a `RobotModel` of `Joint`s, supervising sensor / actuator actors under a `RobotActor` |
| `atomr-physical-ros2` | Bridging onto ROS2 — building a `TopicMap`, binding devices to `Ros2Endpoint`s, the `rclrs` live-bridge feature |
| `atomr-physical-python` | Using the Python overlay — `pip install atomr-physical`, the `atomr_physical.*` module map, building the extension with maturin |
| `atomr-physical-troubleshooting` | Debugging atomr-physical errors — `OutOfRange`, `Ros2Bridge`, `UnitMismatch`, the Phase-2 stub boundaries |

Each `SKILL.md` is a thin router: it points at canonical docs in this
repo (`docs/*.md`) and the relevant crate's API. It deliberately does
**not** restate API surfaces that belong in rustdoc, because those
drift faster than docs.

## Installing

Pick the path that matches your assistant. The skills themselves are
vendor-neutral `SKILL.md` files — only the install mechanism differs.

### Claude Code (recommended: marketplace)

```text
/plugin marketplace add rustakka/atomr-physical
/plugin install atomr-physical-ai-skills@atomr-physical
```

You can also install from a local checkout:

```text
/plugin marketplace add /path/to/atomr-physical
/plugin install atomr-physical-ai-skills@atomr-physical
```

Skills auto-activate based on the `description` frontmatter — no need
to invoke them explicitly.

### Claude Agent SDK / project-local `.claude/skills/`

```bash
# copy (snapshot)
cp -r ai-skills/skills/* .claude/skills/

# symlink (track upstream)
ln -s "$(pwd)/ai-skills/skills/"* .claude/skills/
```

## Stylistic conventions

These skills follow atomr's:

1. **`SKILL.md` + frontmatter** — `name`, `description`. The
   `description` triggers auto-activation, so it is specific about
   *when* to invoke.
2. **Mental model first.** Each skill opens with a one-paragraph mental
   model of the subsystem before diving into API.
3. **Working code blocks.** Snippets compile against the published
   crate version; copy-paste is the intended use.
4. **Pointer to canonical docs.** Every skill ends with a "Canonical
   references" list.
5. **"Common mistakes" coda.** Failure modes the architecture makes
   possible.

## Authoring a new skill

```text
ai-skills/skills/atomr-physical-<topic>/
└── SKILL.md
```

`SKILL.md` frontmatter must include:

```yaml
---
name: atomr-physical-<topic>
description: Use when … . Triggers on … .
---
```

Keep skills focused. If you find yourself documenting two unrelated
subsystems, split them — `atomr-physical-troubleshooting` is the only
deliberately multi-topic one.
