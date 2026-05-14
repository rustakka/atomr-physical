# Release process

This document is the operator-facing reference for cutting an
`atomr-physical` release. It covers the day-to-day workflow, the
architecture that makes auto-publish work, and the recovery playbook.

For workflow-internal detail (job names, matrix entries, build
commands), see [release-pipeline.md](release-pipeline.md).

## Current state: automated execution is OFF

Every workflow is gated to manual `workflow_dispatch` while
atomr-physical is still being built out. The release machinery below
describes how the pipeline behaves **once the auto-triggers are
enabled** (uncomment the trigger block in each workflow header). Until
then, every step is reachable by hand with `gh workflow run`.

## TL;DR (once triggers are live)

* You release by **landing a Conventional-Commit-typed commit on
  `main`**. `feat:`, `fix:`, `perf:`, `revert:`, `!:` (breaking), or a
  `Release-As: x.y.z` footer triggers a bump and a full publish to
  crates.io + PyPI + GitHub Releases. Everything else (`build:`,
  `chore:`, `docs:`, `ci:`, `test:`, `refactor:`, `style:`) is a no-op.
* You do **not** push tags by hand and do **not** dispatch the release
  workflow by hand. The bump-and-tag bot does both.
* If you need to ship something that isn't a real fix/feat, append a
  `Release-As: x.y.z` footer to any commit body to force an exact-
  version bump.

## Conventional Commits → bump mapping

`version-bump.yml` scans every commit subject + body since the last tag
and picks the highest-priority match.

| Commit type / footer | Effect |
|---|---|
| `feat:` (with or without scope) | minor bump → release |
| `fix:` / `perf:` / `revert:` | patch bump → release |
| Any type with `!:` or a `BREAKING CHANGE:` footer | major bump → release |
| `Release-As: x.y.z` footer on any commit | exact-version bump → release |
| `chore:` / `docs:` / `ci:` / `test:` / `refactor:` / `style:` / `build:` | skip — no bump, no release |
| `chore(release): vX.Y.Z` | skip — this is what the bot itself emits |

Decision priority (highest first): `Release-As:` footer → `BREAKING
CHANGE` / `!:` → `feat:` → `fix:` / `perf:` / `revert:` → skip.

Default to `build:` for anything that isn't actively shipping — it is
the safest type (no unintentional release). Switch types only when you
intend to publish.

## The trampoline architecture

### Why the trampoline exists

GitHub Actions has a long-standing safety feature: **events caused by a
workflow that authenticated with the default `GITHUB_TOKEN` do not
trigger other workflows**. So when `version-bump.yml` does `git push
origin --follow-tags`, the resulting tag push is invisible to
`release.yml`'s `on: push: tags` trigger. Releases would silently never
run. This bit the sibling `atomr` repo between v0.6.1 and v0.9.1 — five
tags landed and zero ran `release.yml`. atomr-physical adopts the
trampoline pattern from day one.

### How the trampoline works

After `version-bump.yml` pushes the tag, it runs:

```yaml
- name: Trigger release.yml against the new tag
  if: steps.decide.outputs.kind != 'skip' && github.event.inputs.dry_run != 'true'
  env:
    GH_TOKEN: ${{ secrets.GITHUB_TOKEN }}
    NEW_VERSION: ${{ steps.bump.outputs.version }}
  run: |
    gh workflow run release.yml \
      --ref "v${NEW_VERSION}" \
      -f dry_run=false \
      -f skip_python=false \
      -f skip_crates=false
```

A `workflow_dispatch` event from `gh workflow run` does NOT carry the
`GITHUB_TOKEN`-event-suppression bit, so `release.yml` runs normally.
This requires `actions: write` permission on `version-bump.yml`'s job
(`contents: write` alone is insufficient).

Because atomr-physical currently keeps `release.yml` on
`workflow_dispatch` only, the trampoline is in fact the *primary* path
into the release workflow even before the `push: tags` trigger is
uncommented.

## What the pipeline produces

| Artifact | Where it lands | Built by |
|---|---|---|
| 9 Rust crates | crates.io | `publish-crates`, sequentially in dep order |
| `atomr-physical` binary, 5 platforms | GitHub Release | `build-binaries` matrix |
| Python wheels (6 platforms × CPython 3.10–3.13) | PyPI | `build-wheels` matrix |
| 1 Python sdist | PyPI | `build-sdist` |
| `ai-skills.tar.gz` | GitHub Release | `package-skills` |
| Release notes | GitHub Release | `github-release` (from CHANGELOG.md `[Unreleased]`) |

## Crate publish order

The `publish-crates` job walks crates strictly in dep-order with a 70s
throttle between successful publishes and a 620s exponential-backoff
retry on `429 Too Many Requests` (up to 12 attempts per crate).

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
  9    atomr-physical
```

A crate can publish only when every entry in its `[dependencies]` —
**and** every `[dev-dependencies]` entry, because `cargo publish`
resolves the full graph — is already on crates.io.

### Adding a new publishable crate

1. Add the crate to `[workspace.dependencies]` in the root `Cargo.toml`
   with `version = "X.Y.Z"` matching the workspace version.
2. In the crate's `Cargo.toml`, declare every intra-workspace dep as
   `{ workspace = true }` — never a hand-written `version` literal.
3. Slot it into the earliest layer of `publish-crates` whose prior
   layers have published all its deps (and dev-deps).
4. Update `RELEASING.md` and `docs/release-pipeline.md` to match.
5. Internal-only tooling crates (`xtask`) carry `publish = false`.

## Sibling workspace deps

`atomr-physical` consumes the `atomr` actor runtime through
`[workspace.dependencies]` as a **public crates.io version pin only**.
To bump it:

1. Wait for the new `atomr` version on crates.io.
2. Update the pin in this repo's root `Cargo.toml`.
3. `cargo update -p atomr` (or `cargo update --workspace`).
4. Land it as a `feat:` / `fix:` so the bump bot picks it up.

For local development against an unreleased `atomr` change, use
`[patch.crates-io]` in your personal `~/.cargo/config.toml` rather than
reintroducing path-deps.

## Required setup (one-time)

### crates.io

1. Generate an API token at https://crates.io/me with the
   `publish-update` and `publish-new` scopes.
2. Add it as a repo secret named `CRATES_IO_TOKEN`.

### PyPI Trusted Publishing

For each environment (`pypi` for production, `testpypi` for
rehearsals): create the project on the relevant PyPI host, then
*Manage → Publishing → Add a new publisher → GitHub* with Owner
`rustakka`, Repository `atomr-physical`, Workflow `release.yml`,
Environment `pypi` (or `testpypi`). Both environments must also exist
on the GitHub side (Settings → Environments).

### Workflow permissions

`version-bump.yml` needs `contents: write` (commit, tag, push) and
`actions: write` (dispatch `release.yml`). `release.yml` needs
`contents: write` (the GitHub Release) and `id-token: write` (PyPI
OIDC Trusted Publishing).

## Troubleshooting cookbook

### "auto-bump created a tag but release.yml didn't run"

Verify the trampoline step in `version-bump.yml` exists, has
`actions: write` on the job, and didn't fail. Check the
"Trigger release.yml against the new tag" step's log.

### "publish-crates failed mid-loop on crate N"

The loop treats `already uploaded` as success, so re-running against
the same tag is cheap. Re-dispatch with
`gh workflow run release.yml --ref vX.Y.Z -f dry_run=false`.
Investigate the failed crate — common causes are a stale intra-
workspace pin (use `{ workspace = true }`), a wrong dep-order slot, or
the `atomr` pin pointing at a version not yet on crates.io.

### "failed to select a version for the requirement `atomr = ^0.X.Y`"

The `atomr` sibling pin references a version that isn't on crates.io.
Wait for that `atomr` release, then update the pin.

### "PyPI invalid-publisher"

The Trusted Publisher claims (owner / repo / workflow / environment) on
the PyPI side don't match the run. Re-check the registration —
workflow name is case-sensitive and filename-only.

### "PyPI 400 License-File LICENSE does not exist in distribution file"

`pyproject.toml` already carries the fix:

```toml
[tool.maturin]
include = [{ path = "LICENSE", format = "sdist" }]
```

## When to update this document

* You change `version-bump.yml` or `release.yml` in a way that affects
  the operator-facing surface.
* You add a new publishable crate (update the layer diagram and count).
* You add a new artifact target (wheel platform, binary OS).
* You enable an auto-trigger that was previously commented out.
