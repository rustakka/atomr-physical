//! Parametric two-wheel inverted-pendulum balance controller.
//!
//! [`TwoWheelPendulumController`] linearises the standard
//! inverted-pendulum-on-wheels dynamics around the upright equilibrium,
//! builds an LQR over the four-state model `[θ, θ̇, x, ẋ]`, and exposes
//! a single-input wheel-torque output. The 1-input form is mirrored to
//! both wheel actuators by [`BalanceEngineActor`] for a differential
//! drive — left and right receive the same commanded torque, so the
//! robot balances without steering correction.
//!
//! Linearised model rows:
//! - Row 0 of `A` / `B`: ∂(pitch)/∂t — couples to pitch rate.
//! - Row 1: ∂(pitch_rate)/∂t — gravity-induced pitch acceleration.
//! - Row 2: ∂(position)/∂t — couples to velocity.
//! - Row 3: ∂(velocity)/∂t — wheel torque induced acceleration.
//!
//! Sign convention: positive `pitch_rad` means the body is leaning
//! forward; the controller responds with positive wheel torque to roll
//! the wheels in the same direction so the body returns to upright.

use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_actuation::ActuatorActorRef;
use atomr_physical_core::{Command, ControlMode, PhysicalError, Quantity, Unit};

/// Convenience type alias matching the workspace convention.
type Result<T> = atomr_physical_core::Result<T>;
use nalgebra::{Matrix1, Matrix4, Matrix4x1, Vector4};
use tokio::sync::{broadcast, oneshot};

use crate::imu_reading::ImuReading;
use crate::loop_rate::LoopRate;
use crate::lqr::{BalanceEngine, Lqr, LqrError};

/// Physical parameters of a two-wheel inverted pendulum.
#[derive(Debug, Clone, Copy)]
pub struct PendulumParams {
    /// Wheel radius in metres.
    pub wheel_radius_m: f64,
    /// Body mass (everything above the wheels) in kilograms.
    pub body_mass_kg: f64,
    /// Height of the body's centre of mass above the wheel axis, in
    /// metres.
    pub body_com_height_m: f64,
    /// Body's moment of inertia about the wheel axis, in kg·m².
    pub body_inertia_kgm2: f64,
    /// Gravitational acceleration in m/s², typically `9.81`.
    pub gravity: f64,
}

impl Default for PendulumParams {
    fn default() -> Self {
        Self {
            wheel_radius_m: 0.127,
            body_mass_kg: 15.0,
            body_com_height_m: 0.4,
            body_inertia_kgm2: 0.5,
            gravity: 9.81,
        }
    }
}

/// A two-wheel inverted pendulum controller backed by LQR.
///
/// Construct with [`TwoWheelPendulumController::new`] — that step
/// solves the LQR once. After construction, [`step`](Self::step) is
/// O(state × output) and allocation-free.
#[derive(Debug, Clone, Copy)]
pub struct TwoWheelPendulumController {
    /// Physical parameters used to build the linearised model.
    pub params: PendulumParams,
    lqr: Lqr<4, 1>,
}

impl TwoWheelPendulumController {
    /// Build a controller from parameters and LQR weights.
    ///
    /// `q_diag` is the diagonal of the state-weighting matrix
    /// (`[θ, θ̇, x, ẋ]`); `r_diag` is the diagonal of the
    /// control-weighting matrix (single input).
    pub fn new(
        params: PendulumParams,
        q_diag: [f64; 4],
        r_diag: [f64; 1],
    ) -> std::result::Result<Self, LqrError> {
        let (a, b) = linearised_model(&params);
        let q = Matrix4::from_diagonal(&Vector4::new(q_diag[0], q_diag[1], q_diag[2], q_diag[3]));
        let r = Matrix1::new(r_diag[0]);
        let lqr = Lqr::solve(a, b, q, r)?;
        Ok(Self { params, lqr })
    }

    /// Compute the commanded wheel torque (N·m) for the given state.
    pub fn step(
        &self,
        pitch_rad: f64,
        pitch_rate_rad_s: f64,
        position_m: f64,
        velocity_m_s: f64,
    ) -> f64 {
        let state = Vector4::new(pitch_rad, pitch_rate_rad_s, position_m, velocity_m_s);
        self.lqr.control(&state)[0]
    }

    /// Borrow the underlying LQR (gain, system matrices). Useful for
    /// diagnostics.
    pub fn lqr(&self) -> &Lqr<4, 1> {
        &self.lqr
    }
}

impl BalanceEngine for TwoWheelPendulumController {
    /// `(pitch, pitch_rate, position, velocity)`.
    type State = (f64, f64, f64, f64);
    fn step(&self, state: &Self::State) -> Vec<f64> {
        let (p, pr, x, v) = *state;
        vec![TwoWheelPendulumController::step(self, p, pr, x, v)]
    }
}

/// Linearise the two-wheel inverted-pendulum dynamics about the
/// upright equilibrium. Returns `(A, B)` in `[θ, θ̇, x, ẋ]` form.
///
/// The model used here is the textbook small-angle approximation:
/// pitch acceleration is driven by gravity acting at the COM height,
/// position is the integral of velocity, and velocity reacts directly
/// to wheel torque scaled by the wheel radius and effective inertia.
/// It is not the most accurate model possible — the cross-coupling
/// between body pitch and wheel acceleration is approximated — but it
/// is enough to produce a stabilising gain for sane parameters and is
/// the standard form taught in robotics texts.
fn linearised_model(p: &PendulumParams) -> (Matrix4<f64>, Matrix4x1<f64>) {
    let m = p.body_mass_kg;
    let l = p.body_com_height_m;
    let i_body = p.body_inertia_kgm2.max(1e-9);
    let r = p.wheel_radius_m.max(1e-9);
    let g = p.gravity;

    // Pitch acceleration from gravity (small-angle linearisation).
    let pitch_accel_from_pitch = m * g * l / i_body;
    // Pitch acceleration from wheel torque (acting through the body
    // inertia).
    let pitch_accel_from_torque = -1.0 / i_body;
    // Wheel-induced linear acceleration: torque / (m * r) — treating
    // the wheels' rotational inertia as small relative to body inertia.
    let linear_accel_from_torque = 1.0 / (m * r);

    let a = Matrix4::new(
        0.0, 1.0, 0.0, 0.0, //
        pitch_accel_from_pitch, 0.0, 0.0, 0.0, //
        0.0, 0.0, 0.0, 1.0, //
        0.0, 0.0, 0.0, 0.0,
    );
    let b = Matrix4x1::new(0.0, pitch_accel_from_torque, 0.0, linear_accel_from_torque);
    (a, b)
}

/// A snapshot of a [`BalanceEngineActor`]'s observable state.
#[derive(Debug, Clone, Copy, Default)]
pub struct BalanceSnapshot {
    /// Most recent commanded torque, in N·m.
    pub last_torque: f64,
    /// Most recent pitch angle observed, in radians.
    pub last_pitch: f64,
    /// Number of ticks emitted since startup.
    pub ticks_emitted: u64,
}

/// Supervised wrapper for any [`BalanceEngine`].
///
/// Subscribes to an IMU broadcast on startup, evaluates the engine on
/// every [`LoopRate`] tick, and dispatches the same torque to a left
/// and right wheel actuator. Generic over the engine, so a downstream
/// crate can plug in its own pendulum / Segway / monowheel model.
///
/// The IMU input is supplied as a [`broadcast::Sender`] rather than a
/// `Receiver` so the actor (and any restart) can call `.subscribe()`
/// to get a fresh receiver — `Receiver` itself isn't `Clone` and
/// atomr's Props factory must be re-callable.
pub struct BalanceEngineActor<E>
where
    E: BalanceEngine<State = (f64, f64, f64, f64)> + Clone,
{
    /// The wrapped engine.
    pub engine: E,
    /// IMU broadcast the actor subscribes to on startup.
    pub imu: broadcast::Sender<ImuReading>,
    /// Wheel actuators the torque is sent to (left, right).
    pub left_wheel: ActuatorActorRef,
    /// Right wheel actuator.
    pub right_wheel: ActuatorActorRef,
    /// Tick schedule.
    pub rate: LoopRate,
}

impl<E> BalanceEngineActor<E>
where
    E: BalanceEngine<State = (f64, f64, f64, f64)> + Clone,
{
    /// Promote into a supervised atomr actor.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<BalanceEngineActorRef, ActorSystemError> {
        let BalanceEngineActor {
            engine,
            imu,
            left_wheel,
            right_wheel,
            rate,
        } = self;
        let left = left_wheel;
        let right = right_wheel;
        let props = Props::create(move || BalanceRunner {
            engine: engine.clone(),
            imu: imu.subscribe(),
            left_wheel: left.clone(),
            right_wheel: right.clone(),
            rate,
            last_torque: 0.0,
            last_pitch: 0.0,
            ticks: 0,
            latest: None,
        });
        let inner = system.actor_of(props, name)?;
        Ok(BalanceEngineActorRef { inner })
    }
}

/// Mailbox protocol of a [`BalanceEngineActor`].
pub enum BalanceMsg {
    /// Internal tick — drains the IMU stream, runs the engine,
    /// dispatches torque to both wheels.
    Tick,
    /// Snapshot of observable state.
    Snapshot {
        /// One-shot reply channel.
        reply: oneshot::Sender<BalanceSnapshot>,
    },
    /// Probe health: succeeds if both wheel health checks succeed.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

/// A typed handle to a spawned [`BalanceEngineActor`].
#[derive(Clone)]
pub struct BalanceEngineActorRef {
    inner: ActorRef<BalanceMsg>,
}

impl BalanceEngineActorRef {
    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<BalanceMsg> {
        &self.inner
    }

    /// Ask for an observable-state snapshot.
    pub async fn snapshot(&self) -> Result<BalanceSnapshot> {
        self.inner
            .ask_with(|reply| BalanceMsg::Snapshot { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }

    /// Probe wired-wheel health.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| BalanceMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("control actor ask failed: {e:?}"))
}

/// Internal `Actor` backing a spawned [`BalanceEngineActor`].
struct BalanceRunner<E>
where
    E: BalanceEngine<State = (f64, f64, f64, f64)> + Clone,
{
    engine: E,
    imu: broadcast::Receiver<ImuReading>,
    left_wheel: ActuatorActorRef,
    right_wheel: ActuatorActorRef,
    rate: LoopRate,
    last_torque: f64,
    last_pitch: f64,
    ticks: u64,
    latest: Option<ImuReading>,
}

#[async_trait]
impl<E> Actor for BalanceRunner<E>
where
    E: BalanceEngine<State = (f64, f64, f64, f64)> + Clone,
{
    type Msg = BalanceMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        let me = ctx.self_ref().clone();
        let rate = self.rate;
        tokio::spawn(async move {
            let mut interval = rate.interval();
            interval.tick().await;
            loop {
                interval.tick().await;
                if me.is_terminated() {
                    break;
                }
                me.tell(BalanceMsg::Tick);
            }
        });
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: BalanceMsg) {
        match msg {
            BalanceMsg::Tick => {
                self.ticks = self.ticks.wrapping_add(1);
                // Drain the broadcast: keep the most recent reading,
                // skip lag errors.
                loop {
                    match self.imu.try_recv() {
                        Ok(reading) => self.latest = Some(reading),
                        Err(broadcast::error::TryRecvError::Empty) => break,
                        Err(broadcast::error::TryRecvError::Lagged(_)) => continue,
                        Err(broadcast::error::TryRecvError::Closed) => break,
                    }
                }
                let Some(reading) = self.latest.clone() else {
                    return;
                };
                // Pitch is the rotation about the body's lateral axis.
                // Using nalgebra's Euler angle decomposition: roll, pitch,
                // yaw. The "pitch" we want for a balancing two-wheeler
                // is the rotation about Y in the body frame.
                let (_roll, pitch, _yaw) = reading.orientation.euler_angles();
                let pitch_rate = reading.angular_velocity.y;
                // We have no position estimator yet; pass zeros so the
                // pitch term dominates. Real systems can extend this
                // to consume wheel-odometry.
                let torque = self.engine.step(&(pitch, pitch_rate, 0.0, 0.0))[0];
                self.last_pitch = pitch;
                self.last_torque = torque;

                let cmd_left = Command::now(
                    self.left_wheel.id().clone(),
                    ControlMode::Effort,
                    Quantity::new(torque, Unit::NewtonMetre),
                );
                if let Err(e) = self.left_wheel.dispatch(cmd_left).await {
                    tracing::warn!(error = %e, "balance: left wheel dispatch failed");
                }
                let cmd_right = Command::now(
                    self.right_wheel.id().clone(),
                    ControlMode::Effort,
                    Quantity::new(torque, Unit::NewtonMetre),
                );
                if let Err(e) = self.right_wheel.dispatch(cmd_right).await {
                    tracing::warn!(error = %e, "balance: right wheel dispatch failed");
                }
            }
            BalanceMsg::Snapshot { reply } => {
                let _ = reply.send(BalanceSnapshot {
                    last_torque: self.last_torque,
                    last_pitch: self.last_pitch,
                    ticks_emitted: self.ticks,
                });
            }
            BalanceMsg::Health { reply } => {
                let result = match self.left_wheel.health_check().await {
                    Ok(()) => self.right_wheel.health_check().await,
                    Err(e) => Err(e),
                };
                let _ = reply.send(result);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::time::Duration;

    use atomr_core::actor::ActorSystem;
    use atomr_physical_actuation::ActuatorActor;
    use atomr_physical_testkit::MockActuator;
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn pendulum_controller_responds_to_pitch_with_torque_sign() {
        let ctrl = TwoWheelPendulumController::new(
            PendulumParams::default(),
            [10.0, 1.0, 1.0, 1.0],
            [1.0],
        )
        .expect("controller");
        let torque_pos = ctrl.step(0.05, 0.0, 0.0, 0.0);
        let torque_neg = ctrl.step(-0.05, 0.0, 0.0, 0.0);
        assert!(torque_pos.abs() > 0.0, "expected non-zero torque");
        // Opposite pitches should produce opposite-signed torques.
        assert!(
            torque_pos.signum() != torque_neg.signum(),
            "torques for opposite pitch should differ in sign (got {torque_pos}, {torque_neg})"
        );
    }

    #[tokio::test]
    async fn balance_engine_actor_emits_commands() {
        let sys = ActorSystem::create("balance-spawn", atomr_config::Config::reference())
            .await
            .unwrap();
        let left_driver = Arc::new(MockActuator::new("wheel-left"));
        let right_driver = Arc::new(MockActuator::new("wheel-right"));
        let left = ActuatorActor::new(left_driver.clone())
            .spawn(&sys, "bal-left")
            .unwrap();
        let right = ActuatorActor::new(right_driver.clone())
            .spawn(&sys, "bal-right")
            .unwrap();

        // Build a synthetic IMU broadcast.
        let (imu_tx, _imu_keepalive) = broadcast::channel::<ImuReading>(16);
        let reading = ImuReading {
            orientation: UnitQuaternion::identity(),
            angular_velocity: Vector3::zeros(),
            linear_acceleration: Vector3::zeros(),
            timestamp_ms: 0,
        };
        // Tip the body forward a little by rotating around y.
        let mut tipped = reading.clone();
        tipped.orientation = UnitQuaternion::from_euler_angles(0.0, 0.1, 0.0);

        let ctrl = TwoWheelPendulumController::new(
            PendulumParams::default(),
            [10.0, 1.0, 1.0, 1.0],
            [1.0],
        )
        .unwrap();

        let actor = BalanceEngineActor {
            engine: ctrl,
            imu: imu_tx.clone(),
            left_wheel: left,
            right_wheel: right,
            rate: LoopRate::new(Duration::from_millis(10)),
        };
        let bal_ref = actor.spawn(&sys, "bal").unwrap();

        // Feed a few readings.
        for _ in 0..10 {
            imu_tx.send(tipped.clone()).unwrap();
            tokio::time::sleep(Duration::from_millis(15)).await;
        }

        let left_log = left_driver.log();
        let right_log = right_driver.log();
        assert!(
            !left_log.is_empty(),
            "expected at least one left-wheel command"
        );
        assert!(
            !right_log.is_empty(),
            "expected at least one right-wheel command"
        );

        let snap = bal_ref.snapshot().await.unwrap();
        assert!(snap.ticks_emitted > 0);

        sys.terminate().await;
    }
}
