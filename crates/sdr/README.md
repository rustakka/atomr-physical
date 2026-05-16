# atomr-physical-sdr

Software-Defined Radio (HackRF One) as a supervised atomr actor.

This crate adapts the pure-Rust [`rs-hackrf`](https://crates.io/crates/rs-hackrf)
driver into the atomr-physical contract: a HackRF becomes an addressable
[`atomr_core::actor::ActorRef`], its IQ stream fans out over a
[`tokio::sync::broadcast`] channel, and (optionally, behind the
`sigmf` feature) its samples are captured to disk in the
[SigMF](https://github.com/sigmf/SigMF) format that GNU Radio,
`inspectrum`, and `gqrx` already read.

## Scope

* **RX**: full coverage — tune, gain, amplifier, antenna bias-T,
  baseband filter, continuous streaming.
* **TX**: **not supported** by `rs-hackrf` 0.4. The actor surfaces a
  `transmit` method that returns an `Unsupported` error so call sites
  fail fast and clearly. When `rs-hackrf` adds TX, the surface here is
  ready to back it.

## Two-form pattern

Like every device adapter in atomr-physical, [`SdrActor`] is usable two
ways:

1. **Direct** — open the driver, call `snapshot(n_samples)`, get back
   an [`IqChunk`]. No actor system needed; useful in tests and
   one-shot scripts.
2. **Supervised** — call `.spawn(&system, name)` to promote it into a
   supervised atomr actor. The returned [`SdrActorRef`] is a typed
   handle: `subscribe()` for the live IQ stream, `tune()` mid-stream,
   `start_rx()` / `stop_rx()`, `transmit()` (currently `Unsupported`).

## Crate layout

```
src/
├── lib.rs       — public surface
├── error.rs     — SdrError → PhysicalError::Fault
├── config.rs    — SdrParams (centre / rate / gains / amp / bias-T)
├── iq.rs        — IqChunk, sequence + timestamp + Arc<[i8]> samples
├── messages.rs  — SdrMsg mailbox protocol
├── driver.rs    — HackRfDriver: rs-hackrf wrapper, impl Device
├── actor.rs     — SdrActor (config) + SdrActorRef (typed handle)
├── runner.rs    — SdrRunner (impl Actor) — mailbox + RX forwarding
└── persist.rs   — SigmfWriter (feature = "sigmf")
```

See `docs/sdr.md` in the repository root for the architecture diagram
and CLI usage.
