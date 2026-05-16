# Software-Defined Radio (HackRF One)

The SDR subsystem extends atomr-physical's input surface from
low-bandwidth [`Reading`](https://docs.rs/atomr-physical-core/latest/atomr_physical_core/struct.Reading.html)
sampling to **streaming I/Q** — interleaved 8-bit complex samples
pulled off a HackRF One over USB and fanned out to every interested
subscriber on a [`tokio::sync::broadcast`] channel.

Like the rest of atomr-physical, the SDR is shaped as a supervised
actor: an [`SdrActor`] wraps a backend (currently [`rs-hackrf`] against
the HackRF One), the same two-form contract every other device actor
uses applies (offline construction + `.spawn(system, name)` promotion
to a live atomr actor), and the on-disk capture format is
[SigMF](https://github.com/sigmf/SigMF) so GNU Radio, `inspectrum`, and
`gqrx` read the result without further conversion.

The crate name is intentionally generic — `atomr-physical-sdr`, not
`atomr-physical-hackrf` — so an RTL-SDR or USRP backend can plug into
the same actor surface in a follow-up without breaking call sites.

## Subsystem layout

- [`crates/sdr/src/driver.rs`](../crates/sdr/src/driver.rs) —
  `HackRfDriver` (the thin async wrapper around `rs_hackrf::HackRf`)
  and the `SdrBackend` trait the actor speaks to. Implements
  [`Device`](https://docs.rs/atomr-physical-core/latest/atomr_physical_core/trait.Device.html)
  but **not** `Sensor` / `Actuator` — the single-`Reading` shape can't
  carry a streaming IQ flow.
- [`crates/sdr/src/actor.rs`](../crates/sdr/src/actor.rs) — `SdrActor`
  (the offline configuration wrapper) and `SdrActorRef` (the typed
  handle returned by `.spawn`). Mirrors the two-form pattern used by
  `SensorActor` and `ProjectionActor`.
- [`crates/sdr/src/runner.rs`](../crates/sdr/src/runner.rs) —
  `SdrRunner` (`impl Actor`). Serialises every operation through the
  mailbox so the driver's state machine doesn't race against
  concurrent control messages.
- [`crates/sdr/src/config.rs`](../crates/sdr/src/config.rs) —
  `SdrParams`: centre / rate / gains / amp / bias-T. Plain
  `Serialize`/`Deserialize` so it loads from a config file or rides an
  RPC without pulling the actor runtime in.
- [`crates/sdr/src/iq.rs`](../crates/sdr/src/iq.rs) — `IqChunk`: the
  streaming unit. Sequence number, host capture timestamp, centre and
  rate at capture time, and the raw `Arc<[i8]>` interleaved buffer.
- [`crates/sdr/src/messages.rs`](../crates/sdr/src/messages.rs) —
  `SdrMsg`, the runner's mailbox protocol. Each public variant carries
  a `oneshot::Sender` for its reply; `RxChunk` is an internal variant
  the streaming forwarder uses to post chunks back through the same
  mailbox as control messages.
- [`crates/sdr/src/persist.rs`](../crates/sdr/src/persist.rs)
  (feature `sigmf`) — `SigmfWriter`: subscribes to the broadcast
  channel and lays a `.sigmf-data` + `.sigmf-meta` pair down on disk.

## Two-form pattern

`SdrActor` is usable two ways:

1. **Direct** — open a driver and call `snapshot(n_samples)` for a
   one-shot capture. No actor runtime required. Useful in tests and
   one-line scripts.

   ```rust
   use std::sync::Arc;
   use atomr_physical_sdr::{HackRfDriver, SdrActor, SdrParams};

   let driver = Arc::new(HackRfDriver::open_first()?);
   let actor = SdrActor::new(driver)
       .with_params(SdrParams::default_rx().with_centre_hz(100_000_000));
   let chunk = actor.snapshot(1 << 18).await?;
   println!("{} sample pairs @ {} Hz", chunk.len_samples(), chunk.sample_rate_hz);
   ```

2. **Supervised** — call `.spawn(&system, name)` to promote the actor
   into a live atomr supervisor subtree. The returned `SdrActorRef` is
   the typed handle: subscribe to `IqChunk`s on a `broadcast` channel,
   `tune()` mid-stream, `start_rx()` / `stop_rx()`.

   ```rust
   let sdr_ref = SdrActor::new(driver).spawn(&system, "hackrf-0")?;
   let mut iq = sdr_ref.subscribe();
   sdr_ref.start_rx().await?;
   while let Ok(chunk) = iq.recv().await {
       // hand the Arc<[i8]> to a demodulator, a SigMF writer, …
   }
   ```

   `SdrActor::auto_start_rx(true)` flips the runner into "open the
   firehose in `pre_start`" — off by default because you usually want
   to subscribe **before** chunks start flowing.

There is no `sample()` method: SDR is inherently streaming. `snapshot`
is the offline equivalent — it drives the backend through one full
start → drain → stop cycle and returns the accumulated chunk.

## The IQ stream

The streaming unit is `IqChunk`:

```rust
pub struct IqChunk {
    pub sequence: u64,
    pub captured_at: DateTime<Utc>,
    pub centre_hz: u64,
    pub sample_rate_hz: u32,
    pub samples: Arc<[i8]>,
}
```

`samples` is `Arc<[i8]>` — a zero-copy, shared view of the interleaved
I/Q bytes (`[I0, Q0, I1, Q1, ...]`) the way HackRF natively delivers
them. Every subscriber — a live consumer, the SigMF writer, a future
ROS2 bridge — clones the `Arc`, not the bytes. No DSP conversion
happens on the wire.

The fan-out channel is a `tokio::sync::broadcast::Sender<IqChunk>`
with a per-actor capacity (`DEFAULT_BROADCAST_CAPACITY = 256`,
overridable via `SdrActor::with_broadcast_capacity`). At 4 MS/s with
~256 KiB chunks (≈32 ms each) the default depth buffers roughly 8 s of
stream for a slow subscriber before the channel starts dropping the
oldest chunks. Subscribers see those drops as `broadcast::error::RecvError::Lagged`.

`sequence` is monotonic per-actor: the runner re-stamps every chunk
before fan-out so subscribers see a single sequence across stop/start
cycles. Counts reset on supervisor restart (a new runner instance
starts at zero).

`captured_at` is **host-clocked** — it's the wall-clock time the
forwarder pulled the chunk off the rs-hackrf USB thread, not a
hardware timestamp. The chunk's *first* sample landed some
milliseconds earlier; treat the field as an "≤ this instant" upper
bound, not a sample-accurate timestamp.

## Parameter surface

`SdrParams` mirrors the HackRF One's documented envelope. Every field
is SI (Hz / dB / bool — no implicit MHz). `SdrParams::validate` runs
before any value reaches the driver.

| Field | Unit | Range | Mid-stream tunable |
|---|---|---|:---:|
| `centre_hz` | Hz | 1 MHz .. 6 GHz | ✓ |
| `sample_rate_hz` | Hz | 2 MS/s .. 20 MS/s | — |
| `baseband_filter_hz` | Hz | `None` = libhackrf auto (75 % of rate) | — |
| `lna_gain_db` | dB | 0 .. 40, multiples of 8 | ✓ |
| `vga_gain_db` | dB | 0 .. 62, multiples of 2 | ✓ |
| `amp_enable` | bool | RF amp (+14 dB) ahead of LNA | ✓ |
| `antenna_port_pwr` | bool | antenna-port bias-T (DC) | — |

The "mid-stream tunable" subset is exactly what
[`rs_hackrf::AsyncReadControlHandle`] exposes. The other fields —
sample rate, baseband filter, antenna port — can only be reconfigured
during an idle window: a `tune()` carrying them is recorded but not
pushed to hardware until the next `stop_rx` → `start_rx` cycle.

> **Warning — bias-T.** `antenna_port_pwr` puts DC on the antenna
> port. Only enable it when the device downstream of the port wants
> DC (a powered LNA, an active antenna). Feeding DC into a passive
> antenna or a spectrum analyser front end is how you damage either
> the SDR or whatever is on the other end of the coax.

## State machine

`rs_hackrf::HackRf::into_streaming_reader` **consumes** the device.
The driver therefore tracks three states:

```
       open                start_rx              stop_rx
            ─────────▶               ─────────▶
   Closed              Idle                       Streaming
            ◀─────────              ◀─────────
       (after stop_rx,    apply / tune_live    re-acquire by serial
        re-open by serial)                     on next start_rx
```

- **`Idle(HackRf)`** — device handle is open and not streaming. Full
  config writes (`apply`) go through the handle directly.
- **`Streaming { control, forwarder }`** — the streaming reader owns
  the device on a dedicated thread; runtime tune / gain go through
  `AsyncReadControlHandle`. Sample rate, baseband filter, and antenna
  port are **not** live-tunable here — rs-hackrf doesn't expose them
  on the control handle.
- **`Closed`** — the resting state after `stop_rx`. The next
  `start_rx` re-opens the device by the serial captured at open time,
  so a stop/start cycle survives a transient USB hiccup.

The supervisor's `post_stop` aborts the forwarder task and calls
`stop_rx` on the backend, so a fault-driven restart cleans up the
device handle deterministically.

## Persistence (SigMF)

Behind the `sigmf` cargo feature (umbrella feature: `sdr-sigmf`)
[`SigmfWriter`](../crates/sdr/src/persist.rs) consumes the broadcast
channel and lays down a [SigMF](https://github.com/sigmf/SigMF)-compatible
pair at a given base path:

- `<base>.sigmf-data` — raw interleaved `ci8_le` samples, byte-for-byte
  the bytes the HackRF emitted.
- `<base>.sigmf-meta` — JSON header (`core:datatype = ci8_le`,
  `core:sample_rate`, `core:version = 1.0.0`, `core:recorder =
  atomr-physical-sdr`, plus optional `core:author` /
  `core:description`).

Writes are atomic from the consumer's perspective: bytes stream to
`<base>.sigmf-data.partial`, and the file is renamed to its final
name only when `SigmfWriter::close` succeeds. The metadata file is
written **last** — its presence is the signal that a recording
completed cleanly. A `.partial` left behind with no `.sigmf-meta` is
the on-disk fingerprint of a crashed or aborted capture; the writer's
`Drop` impl deliberately leaves the partial in place rather than
renaming it.

The `captures[]` array gets one entry per `centre_hz` transition
observed mid-recording. Every time an incoming chunk's centre
frequency differs from the previous one the writer closes out the
previous capture entry and opens a new one anchored at the running
sample offset, so the resulting metadata round-trips through any
SigMF-aware viewer with the tune history intact.

The schema itself is hand-rolled via `serde_json` — SigMF is a small,
stable specification, so a third-party `sigmf` crate dependency buys
nothing in return for the supply-chain risk.

A `persist_until_eos(rx, writer)` convenience drains a broadcast
receiver into a writer until the channel closes and returns the
writer back for an explicit `close()`.

## CLI

The `atomr-physical` binary grew an `sdr` subcommand when built with
`--features sdr`:

```bash
# List the serial numbers of every connected HackRF.
atomr-physical sdr probe

# Print board / firmware / USB info for the first available HackRF.
atomr-physical sdr info

# Capture 2 s at 100 MHz / 4 MS/s with modest gain.
atomr-physical sdr rx --centre 100M --rate 4M \
    --gain-lna 16 --gain-vga 20 --duration-ms 2000

# Same, but write a SigMF pair to ./fm.sigmf-data + ./fm.sigmf-meta
# (requires --features sdr-sigmf).
atomr-physical sdr rx --centre 100M --rate 4M \
    --duration-ms 2000 --out ./fm

# TX is on the surface but returns Unsupported on rs-hackrf 0.4.
atomr-physical sdr tx --centre 433.92M --rate 8M --file burst.ci8
```

`--centre` and `--rate` accept SI suffixes (`100M`, `2.4G`, `4M`) or
plain Hz integers.

## What's not (yet) here

- **TX** — `rs-hackrf` 0.4 is RX-only. `SdrActor::transmit` and the
  `sdr tx` CLI return `SdrError::Unsupported`. The signature is on the
  surface today so call sites can integrate against it and won't break
  when upstream lands TX.
- **Sweep mode** — HackRF's hardware FFT sweep needs its own state
  machine (a sweep doesn't fit the `Idle ↔ Streaming` flow because the
  device retunes between every FFT bin). A separate `SdrSweepActor`
  will sit alongside this one when there's a concrete consumer for it.
- **ROS2 bridging** — IQ has no native `sensor_msgs` shape, and a
  4 MS/s `ci8_le` firehose isn't a topic anyone wants on a default
  ROS 2 graph. A sister crate `atomr-physical-sdr-ros2` will provide
  a downsampled / framed bridge when the schema settles.
- **Python bindings** — the `Arc<[i8]>` IQ buffer doesn't have a
  zero-copy NumPy view yet, and `IqChunk` isn't `PyClass`'d. Lands
  with `atomr-physical-py-bindings` once an SDR consumer in Python
  actually needs it.
- **Multi-device coordination** — one `SdrActor` is one HackRF. Two
  HackRFs is two independent actors with no shared clock, no
  cross-device tune, no synchronised capture. A coordinated multi-SDR
  supervisor is a separate problem.

## Hardware checklist

To exercise the live path you need:

- **A HackRF One.** Other rs-hackrf-compatible boards (Jawbreaker,
  Rad1o) may work but aren't tested here.
- **USB 3.0 ideally.** USB 2.0 caps at ~20 MB/s, which is barely
  enough headroom for 10 MS/s `ci8_le`. At 20 MS/s you'll drop chunks
  on USB 2.0; the driver logs them but the supervisor doesn't fault.
- **udev rules so a non-root user can claim the device.** The
  upstream [hackrf udev rules](https://github.com/greatscottgadgets/hackrf/blob/master/host/libhackrf/53-hackrf.rules)
  install to `/etc/udev/rules.d/53-hackrf.rules` and grant the
  `plugdev` group access — without them the rs-hackrf `open` call
  fails with a USB permission error.

## See also

- [Projection](projection.md) — the output-side companion subsystem
  (Sunshine/Moonlight); shares the supervised-actor + two-form pattern.
- [ROS2 bridge](ros2-bridge.md) — the topic-graph mapping for the
  sensor / actuator surface; the SDR is intentionally out of scope.
- [Architecture](architecture.md) — the crate stack and the
  device-actor model the SDR subsystem plugs into.
- [`rs-hackrf` on crates.io](https://crates.io/crates/rs-hackrf) — the
  pure-Rust HackRF driver this crate adapts.
- [SigMF specification](https://github.com/sigmf/SigMF) — the on-disk
  capture format.
