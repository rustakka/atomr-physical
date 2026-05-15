//! Sensor-side actors for atomr-physical.
//!
//! A hardware driver implements [`atomr_physical_core::Sensor`] in plain
//! async Rust. This crate adapts that implementation into a supervised
//! atomr actor — [`SensorActor`] — that owns a sampling loop, applies a
//! [`Calibration`], and (since 0.2.0) publishes [`Reading`]s onto a
//! [`tokio::sync::broadcast`] channel so downstream subscribers see the
//! reading stream over a mailbox-decoupled fan-out.
//!
//! Two ways to use a [`SensorActor`]:
//!
//! 1. **Direct** — construct it and call [`SensorActor::sample`]. No
//!    runtime; useful in tests and one-shot reads.
//! 2. **Supervised** — call [`SensorActor::spawn`] to promote it into a
//!    live atomr actor under an [`atomr_core::actor::ActorSystem`]. The
//!    returned [`SensorActorRef`] is a typed handle to the mailbox and
//!    the broadcast fan-out.
//!
//! The atomr actor runtime is re-exported as [`actor`] so downstream
//! crates have a single import path for it.

use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{PhysicalError, Reading, Result, Sensor, SensorId};
use serde::{Deserialize, Serialize};
use tokio::sync::{broadcast, oneshot};

/// Re-export of the atomr actor runtime this crate builds on.
pub use atomr_core as actor;

/// How often a [`SensorActor`] should poll its underlying [`Sensor`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SamplingPolicy {
    /// Poll on a fixed period.
    FixedRate {
        /// The polling period, in milliseconds.
        period_ms: u64,
    },
    /// Take a reading only when explicitly asked (request / response).
    OnDemand,
}

impl SamplingPolicy {
    /// A 10 Hz fixed-rate policy — a sane default for most chassis
    /// sensors.
    pub fn default_rate() -> Self {
        SamplingPolicy::FixedRate { period_ms: 100 }
    }

    /// The sampling period as a [`Duration`], if this policy is
    /// rate-based.
    pub fn period(&self) -> Option<Duration> {
        match self {
            SamplingPolicy::FixedRate { period_ms } => Some(Duration::from_millis(*period_ms)),
            SamplingPolicy::OnDemand => None,
        }
    }
}

/// A linear calibration applied to a raw sensor value:
/// `corrected = raw * scale + offset`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Calibration {
    /// Multiplicative scale factor.
    pub scale: f64,
    /// Additive offset, applied after scaling.
    pub offset: f64,
}

impl Calibration {
    /// The identity calibration — passes raw values through unchanged.
    pub fn identity() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
        }
    }

    /// Apply the calibration to a raw value.
    pub fn apply(&self, raw: f64) -> f64 {
        raw * self.scale + self.offset
    }
}

impl Default for Calibration {
    fn default() -> Self {
        Self::identity()
    }
}

/// Adapts a [`Sensor`] driver into a supervised atomr actor.
///
/// Construct one with [`SensorActor::new`], attach an optional
/// [`Calibration`] with [`with_calibration`](Self::with_calibration), and
/// either call [`sample`](Self::sample) directly for hardware-free tests
/// or [`spawn`](Self::spawn) (or [`spawn_under`](Self::spawn_under) for
/// a child under a parent actor) to promote it to a live, supervised
/// actor running on an [`ActorSystem`].
///
/// `Clone` is cheap — the only non-`Copy` field is the `Arc<dyn Sensor>`
/// — so the same configuration can both be sampled directly and spawned
/// as an actor.
#[derive(Clone)]
pub struct SensorActor {
    sensor: Arc<dyn Sensor>,
    policy: SamplingPolicy,
    calibration: Calibration,
}

impl SensorActor {
    /// Wrap a sensor driver with a sampling policy.
    pub fn new(sensor: Arc<dyn Sensor>, policy: SamplingPolicy) -> Self {
        Self {
            sensor,
            policy,
            calibration: Calibration::identity(),
        }
    }

    /// Builder-style: attach a calibration applied to every reading.
    pub fn with_calibration(mut self, calibration: Calibration) -> Self {
        self.calibration = calibration;
        self
    }

    /// The id of the wrapped sensor.
    pub fn id(&self) -> SensorId {
        SensorId::from(self.sensor.descriptor().id.as_str())
    }

    /// This actor's sampling policy.
    pub fn policy(&self) -> SamplingPolicy {
        self.policy
    }

    /// This actor's calibration.
    pub fn calibration(&self) -> Calibration {
        self.calibration
    }

    /// Take one calibrated reading directly from the underlying driver.
    ///
    /// This bypasses the mailbox. After [`spawn`](Self::spawn) the same
    /// effect is available through [`SensorActorRef::sample`].
    pub async fn sample(&self) -> Result<Reading> {
        let mut reading = self.sensor.read().await?;
        reading.quantity.value = self.calibration.apply(reading.quantity.value);
        Ok(reading)
    }

    /// Promote this sensor into a supervised atomr actor under `system`,
    /// registered at `name`. Returns a [`SensorActorRef`] — a typed
    /// handle to the mailbox and the broadcast stream.
    ///
    /// On [`SamplingPolicy::FixedRate`] the actor schedules a periodic
    /// `Tick` self-message; each tick reads through the driver, applies
    /// the calibration, and publishes the [`Reading`] to all
    /// [`SensorActorRef::subscribe`]rs.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<SensorActorRef, ActorSystemError> {
        let (props, broadcast_tx, id) = self.into_runner_props();
        let actor_ref = system.actor_of(props, name)?;
        Ok(SensorActorRef {
            inner: actor_ref,
            broadcast_tx,
            id,
        })
    }

    /// Promote this sensor into a supervised atomr actor as a **child**
    /// of the parent actor `P` whose [`Context`] is `ctx`. Used by
    /// [`atomr_physical_robotics::RobotActor`] to build its sensor /
    /// actuator subtree.
    ///
    /// Returns [`PhysicalError::Fault`] if atomr refuses the spawn (e.g.
    /// duplicate child name). The underlying `SpawnError` type isn't
    /// reachable through atomr-core 0.9.2's public surface, so we
    /// stringify it at the boundary.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<SensorActorRef> {
        let (props, broadcast_tx, id) = self.into_runner_props();
        let actor_ref = ctx
            .spawn(props, name)
            .map_err(|e| PhysicalError::Fault(format!("sensor child spawn failed: {e}")))?;
        Ok(SensorActorRef {
            inner: actor_ref,
            broadcast_tx,
            id,
        })
    }

    fn into_runner_props(self) -> (Props<SensorRunner>, broadcast::Sender<Reading>, SensorId) {
        let id = self.id();
        let (broadcast_tx, _) = broadcast::channel(BROADCAST_CAPACITY);
        let bx = broadcast_tx.clone();
        let sensor = self.sensor;
        let calibration = self.calibration;
        let policy = self.policy;
        let props = Props::create(move || SensorRunner {
            sensor: sensor.clone(),
            calibration,
            policy,
            broadcast_tx: bx.clone(),
        });
        (props, broadcast_tx, id)
    }
}

/// Buffer depth of the per-sensor broadcast channel — when a slow
/// subscriber lags more than this many readings, it sees a
/// `RecvError::Lagged` and skips ahead. Sized to a couple of seconds
/// at the default 10 Hz sampling rate.
const BROADCAST_CAPACITY: usize = 64;

/// The mailbox protocol of a live [`SensorActor`].
///
/// Construct messages through [`SensorActorRef`] rather than reaching
/// for the variants directly; the helpers wrap the oneshot replies and
/// the ask timeout.
pub enum SensorMsg {
    /// Take one calibrated reading and reply over `reply`.
    Sample {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<Reading>>,
    },
    /// Internal tick fired by the periodic sampling loop. Drains a
    /// calibrated reading and publishes it on the broadcast channel.
    Tick,
    /// Run the driver's health check and reply.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

/// A typed handle to a spawned [`SensorActor`] — the live, supervised
/// variant of the sensor.
///
/// Cheap to clone; `tell`/`ask` go over the actor's mailbox, and
/// [`subscribe`](Self::subscribe) gets a fresh broadcast receiver for
/// the reading stream.
#[derive(Clone)]
pub struct SensorActorRef {
    inner: ActorRef<SensorMsg>,
    broadcast_tx: broadcast::Sender<Reading>,
    id: SensorId,
}

impl SensorActorRef {
    /// The id of the wrapped sensor.
    pub fn id(&self) -> &SensorId {
        &self.id
    }

    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<SensorMsg> {
        &self.inner
    }

    /// Subscribe to the reading stream. Each ticked reading is fanned
    /// out to every live subscriber.
    pub fn subscribe(&self) -> broadcast::Receiver<Reading> {
        self.broadcast_tx.subscribe()
    }

    /// Ask the actor for one calibrated reading.
    pub async fn sample(&self) -> Result<Reading> {
        self.inner
            .ask_with(|reply| SensorMsg::Sample { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }

    /// Ask the actor to run the driver's health check.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| SensorMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("sensor actor ask failed: {e:?}"))
}

/// Internal Actor implementation backing a spawned [`SensorActor`].
struct SensorRunner {
    sensor: Arc<dyn Sensor>,
    calibration: Calibration,
    policy: SamplingPolicy,
    broadcast_tx: broadcast::Sender<Reading>,
}

#[async_trait]
impl Actor for SensorRunner {
    type Msg = SensorMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        if let SamplingPolicy::FixedRate { period_ms } = self.policy {
            let me = ctx.self_ref().clone();
            let period = Duration::from_millis(period_ms);
            tokio::spawn(async move {
                let mut interval = tokio::time::interval(period);
                interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
                // skip the immediate-first tick — pre_start needs to return
                // before the first message lands so the actor is in
                // Running by the time Tick is delivered.
                interval.tick().await;
                loop {
                    interval.tick().await;
                    if me.is_terminated() {
                        break;
                    }
                    me.tell(SensorMsg::Tick);
                }
            });
        }
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: SensorMsg) {
        match msg {
            SensorMsg::Sample { reply } => {
                let _ = reply.send(self.calibrated_read().await);
            }
            SensorMsg::Tick => {
                if let Ok(reading) = self.calibrated_read().await {
                    // Errors from `broadcast::send` mean no live subscribers
                    // — not a failure condition.
                    let _ = self.broadcast_tx.send(reading);
                } else {
                    tracing::warn!(sensor = %self.sensor.descriptor().id, "tick read failed");
                }
            }
            SensorMsg::Health { reply } => {
                let _ = reply.send(self.sensor.health_check().await);
            }
        }
    }
}

impl SensorRunner {
    async fn calibrated_read(&self) -> Result<Reading> {
        let mut reading = self.sensor.read().await?;
        reading.quantity.value = self.calibration.apply(reading.quantity.value);
        Ok(reading)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_core::actor::ActorSystem;
    use atomr_physical_core::Unit;
    use atomr_physical_testkit::MockSensor;

    #[test]
    fn calibration_is_linear() {
        let cal = Calibration {
            scale: 2.0,
            offset: 1.0,
        };
        assert_eq!(cal.apply(3.0), 7.0);
    }

    #[tokio::test]
    async fn sensor_actor_applies_calibration() {
        let driver = Arc::new(MockSensor::constant("s1", 10.0, Unit::Celsius));
        let actor = SensorActor::new(driver, SamplingPolicy::OnDemand).with_calibration(Calibration {
            scale: 1.0,
            offset: -5.0,
        });
        let reading = actor.sample().await.unwrap();
        assert_eq!(reading.quantity.value, 5.0);
    }

    #[tokio::test]
    async fn spawned_sensor_replies_to_sample() {
        let sys = ActorSystem::create("sensing-sample", atomr_config::Config::reference())
            .await
            .unwrap();
        let driver = Arc::new(MockSensor::constant("s1", 21.0, Unit::Celsius));
        let actor_ref = SensorActor::new(driver, SamplingPolicy::OnDemand)
            .with_calibration(Calibration {
                scale: 1.0,
                offset: 0.5,
            })
            .spawn(&sys, "imu-temp")
            .unwrap();
        let reading = actor_ref.sample().await.unwrap();
        assert_eq!(reading.quantity.value, 21.5);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn spawned_sensor_broadcasts_on_tick() {
        let sys = ActorSystem::create("sensing-broadcast", atomr_config::Config::reference())
            .await
            .unwrap();
        let driver = Arc::new(MockSensor::constant("s1", 42.0, Unit::Scalar));
        let actor_ref = SensorActor::new(driver, SamplingPolicy::FixedRate { period_ms: 20 })
            .spawn(&sys, "tick-sensor")
            .unwrap();
        let mut rx = actor_ref.subscribe();
        // First broadcast should land well inside this window — pre_start
        // skips the immediate tick, so the first publish is ~one period in.
        let reading = tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .unwrap()
            .unwrap();
        assert_eq!(reading.quantity.value, 42.0);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn spawned_sensor_health_check_succeeds() {
        let sys = ActorSystem::create("sensing-health", atomr_config::Config::reference())
            .await
            .unwrap();
        let driver = Arc::new(MockSensor::constant("s1", 0.0, Unit::Scalar));
        let actor_ref = SensorActor::new(driver, SamplingPolicy::OnDemand)
            .spawn(&sys, "h")
            .unwrap();
        actor_ref.health_check().await.unwrap();
        sys.terminate().await;
    }
}
