# Contributing to atomr-physical

Thanks for considering a contribution. atomr-physical follows the same
conventions as [atomr](https://github.com/rustakka/atomr) and
[atomr-agents](https://github.com/rustakka/atomr-agents).

## Quick start

```bash
git clone https://github.com/rustakka/atomr-physical
cd atomr-physical
cargo test --workspace
```

The `atomr` actor runtime is consumed as a **crates.io dependency** —
there is no side-by-side checkout to maintain. To develop against an
unreleased `atomr` change, add a `[patch.crates-io]` override to your
personal `~/.cargo/config.toml` rather than reintroducing path-deps:

```toml
[patch.crates-io]
atomr-core = { path = "../atomr/crates/atomr-core" }
```

## Conventional Commits

Commit subjects drive the release pipeline:

| Subject | Bump |
|---|---|
| `feat: …` | minor |
| `fix: …` | patch |
| `BREAKING CHANGE` body or `feat!:` | major |
| `chore: / docs: / ci: / test: / refactor: / style: / build:` | skip (no release) |

See [`RELEASING.md`](RELEASING.md). Note that automated release
execution is currently **off** — every workflow is `workflow_dispatch`
only until the repo is built out (`RELEASING.md` documents the gating).

## Adding a new feature

1. **Pick the right crate.** Each crate owns one concern — `core` is
   pure data + contract traits, `sensing` / `actuation` own one device
   direction each, `robotics` orchestrates, `ros2` bridges. Cross-
   cutting features land as new crates rather than fattening existing
   ones. See [`docs/architecture.md`](docs/architecture.md).
2. **Keep `core` pure.** `atomr-physical-core` carries no actor-runtime,
   hardware, or ROS2 dependency. The actor wiring lives in the layers
   above it.
3. **Add tests.** Unit tests live alongside the implementation
   (`#[cfg(test)] mod tests`); use `atomr-physical-testkit`'s
   `MockSensor` / `MockActuator` for hardware-free coverage. Python
   smoke tests go under `python/atomr_physical/tests/`.
4. **Document.** New public types get a rustdoc paragraph explaining
   intent. New subsystems get a `docs/<topic>.md` page linked from
   `docs/index.md`.
5. **Ship a skill.** New subsystems get a `SKILL.md` under
   `ai-skills/skills/atomr-physical-<topic>/`. Keep it focused on
   *when* to invoke / *what* to write.

## Style

- `cargo fmt` (rustfmt config in `rustfmt.toml`).
- `cargo clippy --workspace -- -D warnings`.
- atomr's idiomatic-rust principles apply by extension. See
  [`atomr/docs/idiomatic-rust.md`](https://github.com/rustakka/atomr/blob/main/docs/idiomatic-rust.md).
- Physical quantities cross public APIs as `Quantity` (value + `Unit`),
  never bare `f64`s.
- Prefer adding behind a feature flag over breaking an existing
  signature.

## Building the Python extension

The Python facade lives at `python/atomr_physical/`; the native
extension is built from `crates/py-bindings/` via
[maturin](https://www.maturin.rs/):

```bash
pip install maturin
maturin develop -m crates/py-bindings/Cargo.toml
pip install -e ".[dev]"
pytest python/atomr_physical/tests/
```

The Rust side is exercised with the usual workspace test command:

```bash
cargo test --workspace
```

## The ROS2 bridge

`atomr-physical-ros2` builds with **no ROS2 installation** — the
`TopicMap` / `Ros2Endpoint` plan is pure Rust. The `rclrs` feature
(`cargo build -p atomr-physical-ros2 --features rclrs`) links the live
ROS2 client library and requires a ROS2 toolchain on the build host;
it is off by default so the workspace builds anywhere.

## Reporting issues

File at https://github.com/rustakka/atomr-physical/issues with:

- Minimal reproduction.
- `cargo --version` and `rustc --version`.
- Workspace feature flags you're using.
- Whether you're on a real driver or `atomr-physical-testkit` mocks.

## License

By contributing you agree your contributions are licensed under
Apache-2.0.
