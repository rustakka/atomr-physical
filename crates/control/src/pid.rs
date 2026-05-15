//! Single-axis PID controller — both the pure-value form ([`Pid`]) and
//! the supervised form ([`PidActor`]).
//!
//! The pure form does no I/O: callers pass in a [`PidState`] and
//! receive a control output. The supervised form is a fully-fledged
//! atomr actor that subscribes to a [`SensorActorRef`] broadcast,
//! evaluates the PID step on every [`LoopRate`] tick, and dispatches
//! the result as a [`Command`] to an [`ActuatorActorRef`].

use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_actuation::ActuatorActorRef;
use atomr_physical_core::{Command, ControlMode, PhysicalError, Quantity, Reading, Result, Unit};
use atomr_physical_sensing::SensorActorRef;
use tokio::sync::oneshot;

use crate::loop_rate::LoopRate;

/// PID gains and output limits.
///
/// The struct is intentionally `Copy`: it carries only the tuning
/// constants. The integrator / last-error state lives in [`PidState`]
/// so the same `Pid` can drive many independent state machines (e.g.
/// per-joint PIDs sharing one set of gains).
#[derive(Debug, Clone, Copy)]
pub struct Pid {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Minimum output value — the controller will not return less.
    pub output_min: f64,
    /// Maximum output value — the controller will not return more.
    pub output_max: f64,
    /// Integrator anti-windup bound: `|integral| ≤ integral_clamp`.
    pub integral_clamp: f64,
}

impl Pid {
    /// Build a PID with the given gains. Output is unbounded
    /// (`±f64::INFINITY`) and integral is unclamped by default.
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
            integral_clamp: f64::INFINITY,
        }
    }

    /// Builder-style: attach output saturation limits.
    pub fn with_output_limits(mut self, min: f64, max: f64) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    /// Builder-style: attach an integrator anti-windup clamp.
    pub fn with_integral_clamp(mut self, clamp: f64) -> Self {
        self.integral_clamp = clamp.abs();
        self
    }

    /// Run one PID step.
    ///
    /// * Error is `setpoint - measurement`.
    /// * Integrator accumulates `error * dt` and is clamped to
    ///   `±integral_clamp`.
    /// * Derivative is `(error − last_error) / dt`. On the first step
    ///   `last_error` is unset and the derivative term contributes
    ///   zero.
    /// * Output is clamped to `[output_min, output_max]`.
    pub fn step(&self, state: &mut PidState, setpoint: f64, measurement: f64, dt: Duration) -> f64 {
        let error = setpoint - measurement;
        let dt_s = dt.as_secs_f64();

        // integrator
        if dt_s > 0.0 {
            state.integral += error * dt_s;
            if state.integral > self.integral_clamp {
                state.integral = self.integral_clamp;
            } else if state.integral < -self.integral_clamp {
                state.integral = -self.integral_clamp;
            }
        }

        // derivative — zero on the first step, since we have no prior
        // error to differentiate against.
        let derivative = if dt_s > 0.0 {
            match state.last_error {
                Some(prev) => (error - prev) / dt_s,
                None => 0.0,
            }
        } else {
            0.0
        };

        state.last_error = Some(error);

        let raw = self.kp * error + self.ki * state.integral + self.kd * derivative;
        raw.clamp(self.output_min, self.output_max)
    }
}

/// Per-loop state for a [`Pid`]: integrator and last error.
#[derive(Debug, Clone, Copy, Default)]
pub struct PidState {
    /// Running integral of error · dt, clamped to the parent
    /// controller's `integral_clamp` each step.
    pub integral: f64,
    /// Error from the previous step, used to compute the derivative
    /// term. `None` until the first step has run.
    pub last_error: Option<f64>,
}

impl PidState {
    /// A fresh state: zeroed integrator, no previous error.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset the state to its initial form. Useful when changing
    /// setpoint by a large amount or when re-engaging the controller
    /// after a manual override.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = None;
    }
}

/// A snapshot of a [`PidActor`]'s observable state. Returned through
/// the supervised form's mailbox.
#[derive(Debug, Clone, Copy, Default)]
pub struct PidSnapshot {
    /// The current setpoint.
    pub setpoint: f64,
    /// Most recent measurement seen on the sensor stream.
    pub last_measurement: f64,
    /// Most recent control output dispatched to the actuator.
    pub last_output: f64,
    /// Integrator value.
    pub integral: f64,
    /// Number of ticks evaluated since startup.
    pub ticks: u64,
}

/// A supervised PID controller wired between a sensor and an actuator.
///
/// Cheap to clone — handles are `Arc`-backed mailboxes.
#[derive(Clone)]
pub struct PidActor {
    /// The tuning constants.
    pub pid: Pid,
    /// Source of measurements.
    pub input: SensorActorRef,
    /// Destination of commands.
    pub output: ActuatorActorRef,
    /// Control mode the command's setpoint is interpreted as.
    pub mode: ControlMode,
    /// The setpoint the controller drives the measurement toward.
    pub setpoint: f64,
    /// How fast the control loop ticks.
    pub rate: LoopRate,
}

impl PidActor {
    /// Promote this controller into a supervised atomr actor.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<PidActorRef, ActorSystemError> {
        let PidActor {
            pid,
            input,
            output,
            mode,
            setpoint,
            rate,
        } = self;
        let props = Props::create(move || PidRunner {
            pid,
            input: input.clone(),
            output: output.clone(),
            mode,
            setpoint,
            rate,
            state: PidState::new(),
            last_measurement: 0.0,
            last_output: 0.0,
            ticks: 0,
        });
        let inner = system.actor_of(props, name)?;
        Ok(PidActorRef { inner })
    }
}

/// The mailbox protocol of a live [`PidActor`].
pub enum PidMsg {
    /// Internal tick — pulls latest measurement, runs PID step,
    /// dispatches command.
    Tick,
    /// Update the setpoint at runtime.
    SetSetpoint(f64),
    /// Take a snapshot of the controller's observable state.
    Snapshot {
        /// One-shot reply channel.
        reply: oneshot::Sender<PidSnapshot>,
    },
    /// Probe controller health: succeeds if both sensor and actuator
    /// health checks succeed.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

/// A typed handle to a spawned [`PidActor`].
#[derive(Clone)]
pub struct PidActorRef {
    inner: ActorRef<PidMsg>,
}

impl PidActorRef {
    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<PidMsg> {
        &self.inner
    }

    /// Update the controller setpoint (fire-and-forget).
    pub fn set_setpoint(&self, setpoint: f64) {
        self.inner.tell(PidMsg::SetSetpoint(setpoint));
    }

    /// Ask the controller for a snapshot of its observable state.
    pub async fn snapshot(&self) -> Result<PidSnapshot> {
        self.inner
            .ask_with(|reply| PidMsg::Snapshot { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }

    /// Run the controller's wired health check (sensor + actuator).
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| PidMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("control actor ask failed: {e:?}"))
}

/// Internal `Actor` implementation backing a spawned [`PidActor`].
struct PidRunner {
    pid: Pid,
    input: SensorActorRef,
    output: ActuatorActorRef,
    mode: ControlMode,
    setpoint: f64,
    rate: LoopRate,
    state: PidState,
    last_measurement: f64,
    last_output: f64,
    ticks: u64,
}

#[async_trait]
impl Actor for PidRunner {
    type Msg = PidMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        let me = ctx.self_ref().clone();
        let rate = self.rate;
        tokio::spawn(async move {
            let mut interval = rate.interval();
            // Skip the immediate first tick so pre_start can return
            // before the first message lands — matches the sensing
            // crate's convention.
            interval.tick().await;
            loop {
                interval.tick().await;
                if me.is_terminated() {
                    break;
                }
                me.tell(PidMsg::Tick);
            }
        });
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: PidMsg) {
        match msg {
            PidMsg::Tick => {
                self.ticks = self.ticks.wrapping_add(1);
                let measurement = match self.input.sample().await {
                    Ok(Reading { quantity, .. }) => quantity.value,
                    Err(e) => {
                        tracing::warn!(error = %e, "pid tick: sensor sample failed");
                        return;
                    }
                };
                self.last_measurement = measurement;
                let dt = self.rate.period;
                let output = self.pid.step(&mut self.state, self.setpoint, measurement, dt);
                self.last_output = output;
                let cmd = Command::now(
                    self.output.id().clone(),
                    self.mode,
                    Quantity::new(output, effort_unit(self.mode)),
                );
                if let Err(e) = self.output.dispatch(cmd).await {
                    tracing::warn!(error = %e, "pid tick: actuator dispatch failed");
                }
            }
            PidMsg::SetSetpoint(new_setpoint) => {
                self.setpoint = new_setpoint;
            }
            PidMsg::Snapshot { reply } => {
                let _ = reply.send(PidSnapshot {
                    setpoint: self.setpoint,
                    last_measurement: self.last_measurement,
                    last_output: self.last_output,
                    integral: self.state.integral,
                    ticks: self.ticks,
                });
            }
            PidMsg::Health { reply } => {
                let result = match self.input.health_check().await {
                    Ok(()) => self.output.health_check().await,
                    Err(e) => Err(e),
                };
                let _ = reply.send(result);
            }
        }
    }
}

/// Map a [`ControlMode`] to a sensible default [`Unit`] for the
/// outgoing setpoint quantity. Falls back to scalar for control modes
/// added after this crate's MSRV (the enum is `#[non_exhaustive]`).
fn effort_unit(mode: ControlMode) -> Unit {
    match mode {
        ControlMode::Position => Unit::Radian,
        ControlMode::Velocity => Unit::RadianPerSecond,
        ControlMode::Effort => Unit::NewtonMetre,
        ControlMode::Duty => Unit::Percent,
        _ => Unit::Scalar,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::time::Duration;

    use atomr_core::actor::ActorSystem;
    use atomr_physical_actuation::ActuatorActor;
    use atomr_physical_sensing::{SamplingPolicy, SensorActor};
    use atomr_physical_testkit::{MockActuator, MockSensor};

    #[test]
    fn zero_error_zero_output() {
        let pid = Pid::new(1.0, 0.0, 0.0);
        let mut state = PidState::new();
        let out = pid.step(&mut state, 5.0, 5.0, Duration::from_millis(10));
        assert_eq!(out, 0.0);
    }

    #[test]
    fn positive_error_positive_output_clamped() {
        let pid = Pid::new(10.0, 0.0, 0.0).with_output_limits(-1.0, 1.0);
        let mut state = PidState::new();
        let out = pid.step(&mut state, 5.0, 0.0, Duration::from_millis(10));
        // kp * error = 50, clamped to 1.0
        assert_eq!(out, 1.0);
    }

    #[test]
    fn integral_clamps_to_bound() {
        let pid = Pid::new(0.0, 1.0, 0.0).with_integral_clamp(2.0);
        let mut state = PidState::new();
        // 5 steps of error=10 at dt=1s -> raw integral 50, clamped 2.
        for _ in 0..5 {
            pid.step(&mut state, 10.0, 0.0, Duration::from_secs(1));
        }
        assert_eq!(state.integral, 2.0);
    }

    #[test]
    fn first_step_derivative_is_zero() {
        // kp=0, ki=0, kd=1: any non-zero derivative would dominate the
        // output. Since last_error is None on the first step, the
        // derivative is 0 and the output is 0.
        let pid = Pid::new(0.0, 0.0, 1.0);
        let mut state = PidState::new();
        let out = pid.step(&mut state, 5.0, 0.0, Duration::from_secs(1));
        assert_eq!(out, 0.0);
    }

    #[tokio::test]
    async fn pid_actor_drives_actuator_toward_setpoint() {
        let sys = ActorSystem::create("pid-converge", atomr_config::Config::reference())
            .await
            .unwrap();
        let sensor_driver = Arc::new(MockSensor::constant("s1", 0.0, Unit::Scalar));
        let sensor_ref = SensorActor::new(sensor_driver, SamplingPolicy::OnDemand)
            .spawn(&sys, "s-pid")
            .unwrap();
        let actuator_driver = Arc::new(MockActuator::new("a1"));
        let actuator_ref = ActuatorActor::new(actuator_driver.clone())
            .spawn(&sys, "a-pid")
            .unwrap();

        let actor = PidActor {
            pid: Pid::new(1.0, 0.0, 0.0).with_output_limits(-10.0, 10.0),
            input: sensor_ref,
            output: actuator_ref,
            mode: ControlMode::Effort,
            setpoint: 1.0,
            rate: LoopRate::new(Duration::from_millis(10)),
        };
        let pid_ref = actor.spawn(&sys, "pid-1").unwrap();

        tokio::time::sleep(Duration::from_millis(200)).await;

        let log = actuator_driver.log();
        assert!(
            !log.is_empty(),
            "PID actor should have issued at least one command"
        );
        // With kp=1 and error=1.0 (setpoint − measurement), the
        // recorded commands should be ~1.0 and clamped to the limit.
        let last_value = log.last().unwrap().setpoint.value;
        assert!(
            (last_value - 1.0).abs() < 1e-6,
            "expected last command near setpoint 1.0, got {last_value}"
        );

        // Snapshot still works.
        let snap = pid_ref.snapshot().await.unwrap();
        assert!(snap.ticks > 0);

        sys.terminate().await;
    }
}
