# Release pipeline

> **See also:** [release-process.md](release-process.md) — the
> operator-facing reference (how to ship, conventional-commit rules,
> trampoline architecture, troubleshooting). This document focuses on
> workflow internals: jobs, matrix entries, build commands.

## Status: automated execution is OFF

Every workflow in `.github/workflows/` is currently gated to manual
`workflow_dispatch`. The pipeline is fully wired and follows the atomr
release convention, but nothing fires on its own while the repo is
still being built out. Each workflow's header comment flags the exact
trigger block to uncomment when the repo is ready to go live. See
[`RELEASING.md`](../RELEASING.md) for the gating table.

## What `release.yml` ships

`.github/workflows/release.yml` ships atomr-physical to three places on
every release run:

1. **GitHub Releases** — pre-built `atomr-physical` binaries, all built
   Python wheels, and the `ai-skills.tar.gz` marketplace artifact.
2. **crates.io** — every publishable Rust crate (9 in total), in
   dependency order.
3. **PyPI** — platform-specific wheels (Linux glibc x86_64/aarch64,
   Linux musl x86_64/aarch64, macOS universal2, Windows x86_64) and an
   sdist.

## Triggering

Once the triggers are enabled, there are three paths into the pipeline;
they all converge on the same publish jobs.

* **Direct tag push** (`git push origin vX.Y.Z`) — fires
  `on: push: tags` (commented out by default).
* **Auto-bump trampoline** — `version-bump.yml` runs on every push to
  `main`, decides a SemVer bump from Conventional-Commit subjects, and
  on a non-skip decision commits the bump, tags it, pushes, **and
  explicitly dispatches `release.yml`** via `gh workflow run
  release.yml --ref vX.Y.Z -f dry_run=false`. The explicit dispatch is
  required because tag events authored by the default `GITHUB_TOKEN` do
  not fire downstream workflows.
* **Manual `workflow_dispatch`** — choose `dry_run=true` for a
  rehearsal that publishes to TestPyPI and runs `cargo publish
  --dry-run`. Toggle `skip_python` / `skip_crates` to ship to only one
  registry.

## What gets built

### Binaries (`build-binaries`)

Single binary `atomr-physical` (from `crates/cli`). Cross-compiled for:

| OS | Target | Notes |
|---|---|---|
| Ubuntu | `x86_64-unknown-linux-gnu` | native cargo |
| Ubuntu (ARM runner) | `aarch64-unknown-linux-gnu` | native cargo on `ubuntu-22.04-arm` |
| macOS | `x86_64-apple-darwin` | native cargo |
| macOS | `aarch64-apple-darwin` | native cargo |
| Windows | `x86_64-pc-windows-msvc` | native cargo |

### Wheels (`build-wheels`)

Built via `PyO3/maturin-action` from `crates/py-bindings/Cargo.toml`.
The action's `--interpreter` flag builds a wheel per CPython ABI
(3.10–3.13).

| OS | Target | Wheel tag |
|---|---|---|
| Ubuntu | `x86_64-unknown-linux-gnu` | `manylinux_2_17_x86_64` |
| Ubuntu (ARM) | `aarch64-unknown-linux-gnu` | `manylinux_2_17_aarch64` |
| Ubuntu | `x86_64-unknown-linux-musl` | `musllinux_1_2_x86_64` |
| Ubuntu (ARM) | `aarch64-unknown-linux-musl` | `musllinux_1_2_aarch64` |
| macOS | `universal2-apple-darwin` | `macosx_*_universal2` |
| Windows | `x86_64-pc-windows-msvc` | `win_amd64` |

### sdist (`build-sdist`)

A single source distribution `atomr_physical-X.Y.Z.tar.gz`.

### ai-skills (`package-skills`)

A tarball of the `ai-skills/` plugin marketplace folder, attached to
the GitHub Release.

## Required secrets / config

| Secret / variable | Where | Used by |
|---|---|---|
| `CRATES_IO_TOKEN` | repo `Settings → Secrets → Actions` | `publish-crates` |
| PyPI Trusted Publisher | configured on PyPI itself, **not** a GitHub secret | `publish-pypi` |
| `ATOMR_PHYSICAL_PUBLISH_ALLOWLIST` (optional) | repo `Settings → Variables` | `publish-crates` (overrides default order) |

### PyPI Trusted Publishing setup

1. Create the project on https://pypi.org/manage/projects/ (or upload
   one wheel manually first).
2. *Manage → Publishing → Add a new publisher → GitHub*.
3. Fill in: Owner `rustakka`, Repository `atomr-physical`, Workflow
   `release.yml`, Environment `pypi`.
4. Repeat for TestPyPI with environment `testpypi`.

## Crates published

The `publish-crates` job walks every publishable crate in dependency
order. `testkit` publishes right after `core` because `sensing`,
`actuation`, and `robotics` carry it as a dev-dependency.

1. `atomr-physical-core`
2. `atomr-physical-testkit`
3. `atomr-physical-sensing`
4. `atomr-physical-actuation`
5. `atomr-physical-robotics`
6. `atomr-physical-ros2`
7. `atomr-physical-py-bindings` (PyO3 cdylib; depends on every internal crate)
8. `atomr-physical-cli` (binary)
9. `atomr-physical` (umbrella; published last)

Workspace members deliberately excluded: `xtask` (`publish = false`).

Adding a new crate? Declare every intra-workspace dep as
`{ workspace = true }` (NOT a hand-written `version = "..."` literal),
slot it into the earliest layer whose prerequisites have all published,
and update `RELEASING.md` + `docs/release-process.md` to match.

## Sibling workspace deps

`atomr-physical` depends on the sibling `atomr` actor runtime declared
in `[workspace.dependencies]` as a **crates.io version pin only**.
There are no `path = "../atomr"` links and the release workflow does
not `actions/checkout` any sibling repo — bumping the `atomr` pin
requires that version to already be on crates.io, full stop.

## Verifying a release locally

```
gh workflow run release.yml -f dry_run=true
```

Runs the verify gate, builds every binary + wheel, runs `cargo publish
--dry-run` on the full workspace, and uploads to TestPyPI without
touching crates.io or production PyPI.
