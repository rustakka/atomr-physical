# Releasing atomr-physical

Quick reference for cutting a release. The deep dives live in:

* [`docs/release-pipeline.md`](docs/release-pipeline.md) — workflow
  internals (jobs, matrix, build commands).
* [`docs/release-process.md`](docs/release-process.md) — operator-
  facing reference (trampoline architecture, troubleshooting,
  Conventional-Commit rules).

## ⚠️ Automated execution is currently OFF

atomr-physical ships with the full atomr release pipeline wired up, but
**every workflow is gated to manual `workflow_dispatch` while the repo
is still being built out** — there is no automated CI execution yet.

| Workflow | Auto-trigger when live | Currently |
|---|---|---|
| `ci.yml` | `push` to `main`, `pull_request` | `workflow_dispatch` only |
| `docs.yml` | `push` to `main` | `workflow_dispatch` only |
| `version-bump.yml` | `push` to `main` (auto-release trampoline) | `workflow_dispatch` only |
| `release.yml` | `push` of a `v*` tag | `workflow_dispatch` only |

To go live, uncomment the trigger block flagged in each workflow's
header comment. Until then you can still exercise the whole pipeline by
hand with `gh workflow run` (see [Manual operations](#manual-operations)).

```
Conventional-Commit on main
        │
        ▼
.github/workflows/version-bump.yml      (workflow_dispatch today)
        │  decides patch / minor / major / skip
        │  bumps Cargo.toml + Cargo.lock + pyproject.toml
        │  commits `chore(release): vX.Y.Z`
        │  tags `vX.Y.Z` and pushes
        │  ⤷ gh workflow run release.yml --ref vX.Y.Z   ← trampoline
        ▼
.github/workflows/release.yml           (workflow_dispatch today)
        │  verify (build + test gate)
        │  build-binaries (5 targets)
        │  build-wheels (6 platforms × CPython 3.10–3.13)
        │  build-sdist
        │  package-skills (ai-skills.tar.gz)
        │  github-release
        │  publish-crates              ← dep-order, allowlist-gated
        │  publish-pypi                ← OIDC trusted publishing
        ▼
   crates.io + PyPI + GitHub Release
```

## Conventional-Commit rules

| Subject prefix | Bump |
|---|---|
| `feat: …` | minor |
| `fix: …` / `perf: …` / `revert: …` | patch |
| `BREAKING CHANGE` body or `!:` after type | major |
| `chore:` / `docs:` / `ci:` / `test:` / `refactor:` / `style:` / `build:` only | skip |

A footer `Release-As: x.y.z` overrides the auto-decision and pins the
exact version.

## Crate publish order (9 crates)

The `publish-crates` job walks every publishable workspace member in
strict dependency order, with a 70s pace between successful publishes
(crates.io rate-limits new crates at ~1/min) and an exponential-backoff
retry on `429 Too Many Requests`.

```
Layer  Crate
─────  ──────────────────────────────────────────────────────────────
  1    atomr-physical-core
  2    atomr-physical-testkit       (sensing/actuation/robotics dev-dep)
  3    atomr-physical-sensing
  4    atomr-physical-actuation
  5    atomr-physical-robotics
  6    atomr-physical-ros2
  7    atomr-physical-py-bindings
  8    atomr-physical-cli
  9    atomr-physical               (umbrella; published last)
```

`xtask` is `publish = false` and never goes to crates.io.

The repo variable `ATOMR_PHYSICAL_PUBLISH_ALLOWLIST` (space-separated
crate names) overrides the default order. Set it to a subset to ship
only those crates (useful for republish recovery).

## Manual operations

```bash
# Dry-run a release: builds artifacts, dry-run publishes, uploads to TestPyPI.
gh workflow run release.yml -f dry_run=true

# Cargo-only release.
gh workflow run release.yml -f dry_run=true -f skip_python=true

# Wheels-only release.
gh workflow run release.yml -f dry_run=true -f skip_crates=true

# Force a bump kind from version-bump.yml (when commits would otherwise skip).
gh workflow run version-bump.yml -f force=patch     # or minor / major

# Pin to an exact version.
gh workflow run version-bump.yml -f release_as=0.1.0
```

## Pre-flight checklist

Before tagging a release, run the pre-flight locally:

```bash
# 1. Workspace builds clean.
cargo check --workspace --all-features

# 2. All tests pass.
cargo test --workspace

# 3. Each crate dry-runs `cargo publish`.
cargo publish -p atomr-physical-core --dry-run
# … repeat through the publish order …

# 4. Documentation builds.
cargo doc --workspace --no-deps

# 5. Umbrella in three configurations.
cargo build -p atomr-physical
cargo build -p atomr-physical --no-default-features
cargo build -p atomr-physical --features full
```

`cargo xtask release-checklist` prints this list.

## Per-crate metadata requirements

Every publishable crate needs:

- `description`
- `keywords` (≤ 5)
- `categories` (one or two from <https://crates.io/category_slugs>)
- `repository`, `homepage`, `license` (`Apache-2.0`)
- `readme = "../../README.md"`

The workspace `[workspace.package]` supplies `version`, `edition`,
`rust-version`, `license`, `repository`, `homepage`, and `authors`.
Per-crate `Cargo.toml`s add `description`, `keywords`, `categories`,
and `readme`. Intra-workspace deps use `{ workspace = true }`, never a
hand-written `version = "..."` literal.

## Sibling workspace deps

`atomr-physical` consumes the sibling `atomr` actor runtime as a
**public crates.io dependency only** — never a `path = "../atomr"`
link. Bumping the `atomr` version pin requires that version to already
be on crates.io; the release pipeline does not check out sibling repos.
For local development against an unreleased `atomr` change, use a
`[patch.crates-io]` override in your personal `~/.cargo/config.toml`.

## Python (PyPI)

The `atomr-physical` Python wheel is built from
`crates/py-bindings/Cargo.toml` via maturin and published to
<https://pypi.org/p/atomr-physical>. Wheels cover CPython 3.10–3.13
across manylinux + musllinux (x86_64 + aarch64), macOS universal2, and
Windows x86_64. Authentication uses
[PyPI Trusted Publishing](https://docs.pypi.org/trusted-publishers/) —
see `docs/release-process.md` for one-time setup.

## Marketplace (`ai-skills`)

The `package-skills` job tars `ai-skills/` into `ai-skills.tar.gz` and
attaches it to the GitHub Release, so consumers can
`/plugin install atomr-physical-ai-skills@atomr-physical`.

## First release

The first published version is **0.1.0**. The `version-bump.yml` job
refuses to auto-bump from no prior tag — the first release must come
from `workflow_dispatch -f release_as=0.1.0` (or a `Release-As: 0.1.0`
footer on the head commit).

## Yanking

If a release has a critical bug:

```bash
cargo yank --vers x.y.z atomr-physical-<crate>
```

Yank from leaves up (umbrella → cli → py-bindings → ros2 → robotics →
actuation → sensing → testkit → core) so dependent versions don't
briefly fail to resolve.
