//! Joint-state aggregator.
//!
//! [`JointFeedbackAggregator`] resolves a joint's per-axis sensors
//! through a [`RobotActorRef`] and publishes a stream of typed
//! [`JointState`] snapshots on a broadcast. Each tick re-samples the
//! resolved sensors and emits a snapshot — components that weren't
//! resolved are simply `None`.

use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, Context, Props};
use atomr_physical_core::{JointId, PhysicalError, Quantity, Result, SensorId};
use atomr_physical_robotics::RobotActorRef;
use atomr_physical_sensing::SensorActorRef;
use tokio::sync::{broadcast, oneshot};

use crate::loop_rate::LoopRate;

const BROADCAST_CAPACITY: usize = 64;

/// One typed snapshot of a joint's instantaneous state.
#[derive(Debug, Clone)]
pub struct JointState {
    /// The joint this snapshot is for.
    pub joint: JointId,
    /// Position reading.
    pub position: Quantity,
    /// Velocity, if a velocity sensor was bound.
    pub velocity: Option<Quantity>,
    /// Motor current, if a current sensor was bound.
    pub current: Option<Quantity>,
    /// Motor temperature, if a temperature sensor was bound.
    pub temperature: Option<Quantity>,
    /// Sample time, milliseconds since the Unix epoch.
    pub timestamp_ms: i64,
}

/// Aggregator spec — references each axis by `SensorId` so the
/// aggregator can look them up through the robot's mailbox at spawn
/// time.
#[derive(Debug, Clone)]
pub struct JointFeedbackAggregator {
    /// Joint this aggregator publishes for.
    pub joint: JointId,
    /// Required: id of the position sensor.
    pub position: SensorId,
    /// Optional: id of the velocity sensor.
    pub velocity: Option<SensorId>,
    /// Optional: id of the current sensor.
    pub current: Option<SensorId>,
    /// Optional: id of the temperature sensor.
    pub temperature: Option<SensorId>,
    /// Tick schedule.
    pub rate: LoopRate,
}

impl JointFeedbackAggregator {
    /// Promote this aggregator into a supervised atomr actor.
    ///
    /// Resolves each `SensorId` through the robot's mailbox at spawn
    /// time. Missing optional sensors are silently skipped; a missing
    /// position sensor is a hard error.
    pub async fn spawn(
        self,
        system: &ActorSystem,
        robot: &RobotActorRef,
        name: &str,
    ) -> Result<JointFeedbackRef> {
        let JointFeedbackAggregator {
            joint,
            position,
            velocity,
            current,
            temperature,
            rate,
        } = self;
        let position_ref = robot.sensor(&position).await?.ok_or_else(|| {
            PhysicalError::UnknownDevice(format!(
                "joint-feedback: position sensor {position} not registered"
            ))
        })?;
        let velocity_ref = match &velocity {
            Some(id) => Some(robot.sensor(id).await?.ok_or_else(|| {
                PhysicalError::UnknownDevice(format!("joint-feedback: velocity sensor {id} not registered"))
            })?),
            None => None,
        };
        let current_ref = match &current {
            Some(id) => Some(robot.sensor(id).await?.ok_or_else(|| {
                PhysicalError::UnknownDevice(format!("joint-feedback: current sensor {id} not registered"))
            })?),
            None => None,
        };
        let temperature_ref = match &temperature {
            Some(id) => Some(robot.sensor(id).await?.ok_or_else(|| {
                PhysicalError::UnknownDevice(format!(
                    "joint-feedback: temperature sensor {id} not registered"
                ))
            })?),
            None => None,
        };

        let (broadcast_tx, _) = broadcast::channel::<JointState>(BROADCAST_CAPACITY);
        let tx_for_factory = broadcast_tx.clone();
        let joint_for_factory = joint.clone();
        let props = Props::create(move || JointFeedbackRunner {
            joint: joint_for_factory.clone(),
            position: position_ref.clone(),
            velocity: velocity_ref.clone(),
            current: current_ref.clone(),
            temperature: temperature_ref.clone(),
            rate,
            broadcast_tx: tx_for_factory.clone(),
        });
        let inner = system
            .actor_of(props, name)
            .map_err(|e| PhysicalError::Fault(format!("joint-feedback spawn failed: {e}")))?;
        Ok(JointFeedbackRef {
            inner,
            broadcast_tx,
            joint,
        })
    }
}

/// Mailbox protocol of a spawned aggregator.
pub enum JointFeedbackMsg {
    /// Internal tick — gathers a snapshot and broadcasts it.
    Tick,
    /// Probe health: succeeds if every resolved sensor's health check
    /// succeeds.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

/// A typed handle to a spawned [`JointFeedbackAggregator`].
#[derive(Clone)]
pub struct JointFeedbackRef {
    inner: ActorRef<JointFeedbackMsg>,
    broadcast_tx: broadcast::Sender<JointState>,
    joint: JointId,
}

impl JointFeedbackRef {
    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<JointFeedbackMsg> {
        &self.inner
    }

    /// The joint this aggregator publishes for.
    pub fn joint(&self) -> &JointId {
        &self.joint
    }

    /// Subscribe to the joint-state stream.
    pub fn subscribe(&self) -> broadcast::Receiver<JointState> {
        self.broadcast_tx.subscribe()
    }

    /// Probe aggregated health.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| JointFeedbackMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("joint-feedback ask failed: {e:?}"))
}

/// Internal `Actor` backing a spawned [`JointFeedbackAggregator`].
struct JointFeedbackRunner {
    joint: JointId,
    position: SensorActorRef,
    velocity: Option<SensorActorRef>,
    current: Option<SensorActorRef>,
    temperature: Option<SensorActorRef>,
    rate: LoopRate,
    broadcast_tx: broadcast::Sender<JointState>,
}

#[async_trait]
impl Actor for JointFeedbackRunner {
    type Msg = JointFeedbackMsg;

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
                me.tell(JointFeedbackMsg::Tick);
            }
        });
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: JointFeedbackMsg) {
        match msg {
            JointFeedbackMsg::Tick => match self.gather().await {
                Ok(snapshot) => {
                    let _ = self.broadcast_tx.send(snapshot);
                }
                Err(e) => {
                    tracing::warn!(error = %e, "joint feedback tick failed");
                }
            },
            JointFeedbackMsg::Health { reply } => {
                let result = self.health_all().await;
                let _ = reply.send(result);
            }
        }
    }
}

impl JointFeedbackRunner {
    async fn gather(&self) -> Result<JointState> {
        let position = self.position.sample().await?.quantity;
        let velocity = match &self.velocity {
            Some(s) => Some(s.sample().await?.quantity),
            None => None,
        };
        let current = match &self.current {
            Some(s) => Some(s.sample().await?.quantity),
            None => None,
        };
        let temperature = match &self.temperature {
            Some(s) => Some(s.sample().await?.quantity),
            None => None,
        };
        Ok(JointState {
            joint: self.joint.clone(),
            position,
            velocity,
            current,
            temperature,
            timestamp_ms: chrono::Utc::now().timestamp_millis(),
        })
    }

    async fn health_all(&self) -> Result<()> {
        self.position.health_check().await?;
        if let Some(s) = &self.velocity {
            s.health_check().await?;
        }
        if let Some(s) = &self.current {
            s.health_check().await?;
        }
        if let Some(s) = &self.temperature {
            s.health_check().await?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    use atomr_core::actor::ActorSystem;
    use atomr_physical_core::Unit;
    use atomr_physical_robotics::{RobotActor, RobotModel};
    use atomr_physical_sensing::{SamplingPolicy, SensorActor};
    use atomr_physical_testkit::MockSensor;

    #[tokio::test]
    async fn joint_feedback_publishes_position_only() {
        let sys = ActorSystem::create("jf-test", atomr_config::Config::reference())
            .await
            .unwrap();
        let mut robot = RobotActor::new(
            atomr_physical_core::RobotId::from("r1"),
            RobotModel::new(),
        );
        robot.add_sensor(SensorActor::new(
            Arc::new(MockSensor::constant("pos", 0.5, Unit::Radian)),
            SamplingPolicy::OnDemand,
        ));
        let robot_ref = robot.spawn(&sys, "robot").unwrap();

        let agg = JointFeedbackAggregator {
            joint: JointId::from("j1"),
            position: SensorId::from("pos"),
            velocity: None,
            current: None,
            temperature: None,
            rate: LoopRate::new(Duration::from_millis(20)),
        };
        let agg_ref = agg.spawn(&sys, &robot_ref, "jf").await.unwrap();
        let mut rx = agg_ref.subscribe();
        let snapshot = tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .expect("joint feedback timed out")
            .expect("joint feedback recv");
        assert_eq!(snapshot.joint, JointId::from("j1"));
        assert!((snapshot.position.value - 0.5).abs() < 1e-9);
        assert!(snapshot.velocity.is_none());
        assert!(snapshot.current.is_none());
        assert!(snapshot.temperature.is_none());

        sys.terminate().await;
    }
}
