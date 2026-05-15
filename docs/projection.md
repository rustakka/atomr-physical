# Projection (Sunshine / Moonlight)

The projection subsystem extends atomr-physical's output surface from
low-bandwidth [`Command`](https://docs.rs/atomr-physical-core/latest/atomr_physical_core/struct.Command.html)
dispatch to full **video projection** — the live screen of a virtual
display streamed to one or more Moonlight clients via supervised
[Sunshine](https://app.lizardbyte.dev/Sunshine) server processes.

Like the rest of atomr-physical, projection is shaped as a supervised
actor subtree: a `ProjectionActor` is the top of the tree, every
`sunshine` process is a supervised child, and the same two-form
contract every other device actor uses (offline construction +
`.spawn(system, name)` promotion to a live atomr actor) applies.

## Mental model

A `ProjectionActor` is to video output what a `RobotActor` is to motor
output: the supervisor at the top of an output subtree, owning a set
of subsystems that cooperate to keep the output running.

```
                  ProjectionActor                    supervisor
   ┌──────────────────────────────────────────────────┐
   │  ┌────────────────────┐   ┌─────────────────┐    │
   │  │ VkmsDisplayManager │   │  PortAllocator  │    │
   │  └─────────┬──────────┘   └────────┬────────┘    │
   │            │                       │             │
   │            ▼                       ▼             │
   │  ┌──────────────────────────────────────────┐    │
   │  │       SunshineInstanceActor × N          │    │   children
   │  │   (one `sunshine` subprocess each)       │    │
   │  └──────────────────────────────────────────┘    │
   │            ▲                       ▲             │
   │            │                       │             │
   │  ┌─────────┴─────────┐   ┌─────────┴────────┐    │
   │  │  MdnsBroadcaster  │   │ClientProvisioner │    │
   │  │ (_nvstream._tcp)  │   │ (HTTPS pairing)  │    │
   │  └───────────────────┘   └──────────────────┘    │
   └──────────────────────────────────────────────────┘
```

A live Moonlight client on the LAN browses `_nvstream._tcp.local.`,
finds a registered instance, pairs against its self-signed HTTPS API
(announce + PIN), and joins the video stream. Everything above the
Sunshine binary itself is plain Rust; the actor owns the lifecycle,
the supervision, and the failure boundary.

## The four subsystems

### `VkmsDisplayManager` — headless virtual displays

Sunshine streams whatever is on a real X / Wayland display. To produce
a screen without a physical monitor attached, the projection actor
backs each stream by a [`vkms`](https://docs.kernel.org/gpu/vkms.html)
virtual KMS display. `VkmsDisplayManager` probes `/sys/module/vkms`,
loads the kernel module if it isn't already, discovers the backing
DRM card under `/sys/class/drm`, and shells out to `xrandr` to bring
named connectors (`Virtual-1`, `Virtual-2`, …) up and down.

`VkmsDisplayManager::offline()` is the test/CI form: it skips every
shell-out and keeps state purely in an in-memory `active` map, so the
integration test against a `/bin/sleep` Sunshine binary doesn't need
root or a real kernel module loaded.

### `PortAllocator` — stride-shifted port windows

A single Sunshine instance binds a fixed set of TCP and UDP ports
(`47984/47989/48010` TCP, `47998/48000/48002` UDP). To run more than
one instance on the same host, the allocator hands out non-overlapping
**port windows** offset by a stride (default 100) from the Moonlight
base — instance 0 takes the base ports, instance 1 takes `+100`, and
so on. The Sunshine binary only takes a single `port = ...` knob (its
HTTPS API port); it derives the rest at fixed offsets.

### `SunshineInstanceActor` — one supervised subprocess per stream

Each active projection is backed by a `SunshineInstanceActor` — a real
atomr actor whose `pre_start` writes a rendered Sunshine config file
into a per-instance `tempfile::TempDir`, spawns the child process,
splits stdout/stderr into log-pump tasks, and hands the child to an
exit-watcher that fires a self-message on termination.
`SunshineInstanceMsg::Shutdown` issues `SIGTERM` by PID; the actor
records the exit code without panicking and only restarts on
supervisor-driven faults (panics in actor code, not graceful child
exits).

### `MdnsBroadcaster` + `ClientProvisioner` — discovery and pairing

The broadcaster registers each live instance as a fully-qualified
`_nvstream._tcp.local.` service via the
[`mdns-sd`](https://docs.rs/mdns-sd) crate, embedding instance id,
bitrate, resolution, and frame rate in the TXT record. The
provisioner is a `reqwest` client that drives Sunshine's local
`/api/pair`, `/api/pin`, and `/api/unpair` endpoints.

Sunshine ships a freshly-minted self-signed TLS cert per instance, so
the HTTPS client accepts invalid certificates and falls back on
**trust on first use** — the SPKI fingerprint observed on the first
successful pair is captured in the `PairingRecord` and can be
re-checked on subsequent reconnects.

## The offline / supervised contract

Like the device actors, `ProjectionActor` exposes both an **offline**
form and a **supervised** form:

```rust
use std::path::PathBuf;
use atomr_physical_projection::{ProjectionActor, ProjectionSpec};

// Builder form: customise binary, mDNS host label, bandwidth tiers,
// supervisor strategy. test_offline = true short-circuits every
// shell-out (vkms, mDNS, pairing) so the tree runs without root, a
// real kernel module, or a LAN.
let actor = ProjectionActor::new(PathBuf::from("/bin/sleep"))
    .with_test_offline(true)
    .with_mdns_host_label("atomr-demo");

// Supervised form: promote to a live atomr actor.
let projection_ref = actor.spawn(&system, "projection")?;
let handle = projection_ref
    .request_projection(ProjectionSpec::defaults())
    .await?;
```

`ProjectionActorRef::request_projection` allocates a port window,
brings up a virtual display, spawns a `SunshineInstanceActor` under
the supervisor, broadcasts the instance over mDNS, and returns a
`ProjectionHandle` carrying every id the caller needs for follow-up
calls (pairing, teardown, summary).

`with_test_offline(true)` is the path the CLI demo and the integration
test (`crates/projection/tests/projection_actor.rs`) take. Combined
with `/bin/sleep` as the Sunshine binary, the whole pipeline runs
hardware-free.

## Bandwidth tiers

`ProjectionActor::with_bandwidth_thresholds` configures a table of
`BandwidthTier`s — one bitrate per client-count threshold. When the
live client count for an instance crosses a threshold, the supervisor
performs a graceful restart of that instance with the new bitrate
baked into its rendered config. The default ladder is `1 client →
20 Mb/s, 2 clients → 10 Mb/s, 3+ clients → 6 Mb/s`.

## The receiver side: `atomr-physical-projection-client`

The companion
[`atomr-physical-projection-client`](../crates/projection-client/README.md)
crate is the remote-node binary. It runs on a small ARM device
(Raspberry Pi, Jetson Nano) on the same LAN, browses
`_nvstream._tcp.local.` for matching services, completes the pairing
handshake, and `exec`s `moonlight-embedded` so the operator sees the
session in the controlling terminal. It deliberately keeps no
persistent state — every invocation is a fresh pair-and-stream cycle.

The crate README covers cross-compile (`aarch64-unknown-linux-gnu`)
and the bundled systemd unit; this doc only documents the
workstation-side actor.

## The CLI

The `atomr-physical` binary grew a `project` subcommand against an
in-process supervisor:

```bash
# Boot the supervisor and spin up two stub projections, hold for 750 ms.
atomr-physical project demo --count 2 --hold-ms 750

# Start one projection and run the full pair-and-tear-down flow.
atomr-physical project pair --hostname my-pi
```

Both default to `/bin/sleep` as the Sunshine binary and the offline
pathway, so they require no privileges. Set `--sunshine-binary
/usr/bin/sunshine` to drive a real Sunshine install.

## Security note

The pairing client uses
`reqwest::ClientBuilder::danger_accept_invalid_certs(true)` because
each Sunshine instance ships its own self-signed certificate. That is
acceptable for a trusted LAN where the mDNS namespace and the physical
network are under the operator's control. Deployments that need
stronger guarantees should pre-distribute each instance's certificate
out-of-band and pin its SPKI fingerprint on the client — the
`PairingRecord` already records the observed fingerprint, but the
current binary does not enforce pre-pinned values.

## Feature flags and dependencies

The projection subsystem is **opt-in**: the umbrella crate's
`projection` feature pulls in `atomr-physical-projection` along with
its network-heavy dependencies (`reqwest`, `mdns-sd`,
`nix` for `SIGTERM`-by-PID). Default builds stay free of those
dependencies entirely.

```toml
[dependencies]
atomr-physical = { version = "0.1", features = ["projection"] }
```

See [feature-matrix.md](feature-matrix.md) for the full flag table.

## Canonical references

- [`crates/projection/src/lib.rs`](../crates/projection/src/lib.rs) — the
  public re-export surface; every type mentioned here is documented in
  rustdoc.
- [`crates/projection/src/actor.rs`](../crates/projection/src/actor.rs) —
  `ProjectionActor`, `ProjectionActorRef`, `ProjectionMsg`,
  `ProjectionSpec`, `BandwidthTier`, `ProjectionHandle`, `PairingTicket`.
- [`crates/projection/tests/projection_actor.rs`](../crates/projection/tests/projection_actor.rs)
  — the hardware-free integration test; the canonical "what does a
  full session look like" reference.
- [`crates/projection-client/README.md`](../crates/projection-client/README.md)
  — receiver-side binary, cross-compile + systemd notes.

## Common mistakes

- **Forgetting `with_test_offline(true)` outside production.** Without
  it, `request_projection` shells out to `modprobe vkms` and `xrandr`;
  on a non-root CI runner that surfaces as a vague `vkms probe failed`
  fault from the actor.
- **Re-using one `PortAllocator` across two `ProjectionActor`s on the
  same host.** Each actor owns its own allocator — co-resident actors
  would issue overlapping windows. Run one supervisor per host instead.
- **Treating the `ProjectionHandle.mdns_service` string as the
  Sunshine HTTP URL.** It is the fully qualified mDNS service name
  (`atomr-abc12345._nvstream._tcp.local.`), not a `https://` URL —
  resolve it through the broadcaster's TXT record first.
