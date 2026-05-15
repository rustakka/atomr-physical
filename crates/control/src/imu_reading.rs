//! Typed IMU reading and an aggregator that fans ten scalar sensor
//! broadcasts into a single typed IMU broadcast.
//!
//! Many IMUs (e.g. the BNO085) expose their channels as ten
//! independent scalar streams: four quaternion components, three
//! gyroscope axes, three accelerometer axes. The
//! [`ImuAggregator`] knits those streams back together into one typed
//! [`ImuReading`] broadcast that downstream controllers
//! ([`crate::pendulum::BalanceEngineActor`]) can subscribe to.

use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{PhysicalError, Result};
use atomr_physical_sensing::SensorActorRef;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use tokio::sync::{broadcast, oneshot};

use crate::loop_rate::LoopRate;

const BROADCAST_CAPACITY: usize = 64;

/// One typed IMU sample.
#[derive(Debug, Clone)]
pub struct ImuReading {
    /// Body orientation (typically world-to-body rotation).
    pub orientation: UnitQuaternion<f64>,
    /// Angular velocity in rad/s, in body frame.
    pub angular_velocity: Vector3<f64>,
    /// Linear acceleration in m/s², in body frame.
    pub linear_acceleration: Vector3<f64>,
    /// Sample time, milliseconds since the Unix epoch.
    pub timestamp_ms: i64,
}

/// An offline-form aggregator that knits ten scalar sensor broadcasts
/// into one [`ImuReading`] broadcast.
#[derive(Clone)]
pub struct ImuAggregator {
    /// Quaternion W (real) component sensor.
    pub quat_w: SensorActorRef,
    /// Quaternion X component sensor.
    pub quat_x: SensorActorRef,
    /// Quaternion Y component sensor.
    pub quat_y: SensorActorRef,
    /// Quaternion Z component sensor.
    pub quat_z: SensorActorRef,
    /// Gyroscope X-axis sensor.
    pub gyro_x: SensorActorRef,
    /// Gyroscope Y-axis sensor.
    pub gyro_y: SensorActorRef,
    /// Gyroscope Z-axis sensor.
    pub gyro_z: SensorActorRef,
    /// Accelerometer X-axis sensor.
    pub accel_x: SensorActorRef,
    /// Accelerometer Y-axis sensor.
    pub accel_y: SensorActorRef,
    /// Accelerometer Z-axis sensor.
    pub accel_z: SensorActorRef,
    /// Tick schedule.
    pub rate: LoopRate,
}

impl ImuAggregator {
    /// Promote into a supervised atomr actor.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<ImuAggregatorRef, ActorSystemError> {
        let (broadcast_tx, _) = broadcast::channel::<ImuReading>(BROADCAST_CAPACITY);
        let tx_for_factory = broadcast_tx.clone();
        let aggregator = self;
        let props = Props::create(move || ImuRunner {
            agg: aggregator.clone(),
            broadcast_tx: tx_for_factory.clone(),
        });
        let inner = system.actor_of(props, name)?;
        Ok(ImuAggregatorRef {
            inner,
            broadcast_tx,
        })
    }
}

/// Mailbox protocol of an [`ImuAggregator`].
pub enum ImuMsg {
    /// Internal tick — samples all ten sensors, publishes one
    /// [`ImuReading`].
    Tick,
    /// Probe health: succeeds if every child sensor's health check
    /// succeeds.
    Health {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
}

/// A typed handle to a spawned [`ImuAggregator`].
#[derive(Clone)]
pub struct ImuAggregatorRef {
    inner: ActorRef<ImuMsg>,
    broadcast_tx: broadcast::Sender<ImuReading>,
}

impl ImuAggregatorRef {
    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<ImuMsg> {
        &self.inner
    }

    /// Subscribe to the IMU broadcast.
    pub fn subscribe(&self) -> broadcast::Receiver<ImuReading> {
        self.broadcast_tx.subscribe()
    }

    /// Probe aggregated health.
    pub async fn health_check(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| ImuMsg::Health { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("imu aggregator ask failed: {e:?}"))
}

/// Internal `Actor` backing an [`ImuAggregator`].
struct ImuRunner {
    agg: ImuAggregator,
    broadcast_tx: broadcast::Sender<ImuReading>,
}

#[async_trait]
impl Actor for ImuRunner {
    type Msg = ImuMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        let me = ctx.self_ref().clone();
        let rate = self.agg.rate;
        tokio::spawn(async move {
            let mut interval = rate.interval();
            interval.tick().await;
            loop {
                interval.tick().await;
                if me.is_terminated() {
                    break;
                }
                me.tell(ImuMsg::Tick);
            }
        });
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: ImuMsg) {
        match msg {
            ImuMsg::Tick => match self.gather().await {
                Ok(reading) => {
                    let _ = self.broadcast_tx.send(reading);
                }
                Err(e) => {
                    tracing::warn!(error = %e, "imu aggregator tick failed");
                }
            },
            ImuMsg::Health { reply } => {
                let result = self.health_all().await;
                let _ = reply.send(result);
            }
        }
    }
}

impl ImuRunner {
    async fn gather(&self) -> Result<ImuReading> {
        let w = self.agg.quat_w.sample().await?.quantity.value;
        let x = self.agg.quat_x.sample().await?.quantity.value;
        let y = self.agg.quat_y.sample().await?.quantity.value;
        let z = self.agg.quat_z.sample().await?.quantity.value;
        let gx = self.agg.gyro_x.sample().await?.quantity.value;
        let gy = self.agg.gyro_y.sample().await?.quantity.value;
        let gz = self.agg.gyro_z.sample().await?.quantity.value;
        let ax = self.agg.accel_x.sample().await?.quantity.value;
        let ay = self.agg.accel_y.sample().await?.quantity.value;
        let az = self.agg.accel_z.sample().await?.quantity.value;

        let raw = Quaternion::new(w, x, y, z);
        let orientation = UnitQuaternion::try_new(raw, 1e-9).unwrap_or_else(UnitQuaternion::identity);

        Ok(ImuReading {
            orientation,
            angular_velocity: Vector3::new(gx, gy, gz),
            linear_acceleration: Vector3::new(ax, ay, az),
            timestamp_ms: chrono::Utc::now().timestamp_millis(),
        })
    }

    async fn health_all(&self) -> Result<()> {
        for sensor in [
            &self.agg.quat_w,
            &self.agg.quat_x,
            &self.agg.quat_y,
            &self.agg.quat_z,
            &self.agg.gyro_x,
            &self.agg.gyro_y,
            &self.agg.gyro_z,
            &self.agg.accel_x,
            &self.agg.accel_y,
            &self.agg.accel_z,
        ] {
            sensor.health_check().await?;
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
    use atomr_physical_sensing::{SamplingPolicy, SensorActor};
    use atomr_physical_testkit::MockSensor;

    #[tokio::test]
    async fn imu_aggregator_publishes_typed_reading() {
        let sys = ActorSystem::create("imu-agg", atomr_config::Config::reference())
            .await
            .unwrap();
        // Identity quaternion: w=1, x=y=z=0.
        let qw = SensorActor::new(
            Arc::new(MockSensor::constant("imu/qw", 1.0, Unit::Scalar)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-qw")
        .unwrap();
        let qx = SensorActor::new(
            Arc::new(MockSensor::constant("imu/qx", 0.0, Unit::Scalar)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-qx")
        .unwrap();
        let qy = SensorActor::new(
            Arc::new(MockSensor::constant("imu/qy", 0.0, Unit::Scalar)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-qy")
        .unwrap();
        let qz = SensorActor::new(
            Arc::new(MockSensor::constant("imu/qz", 0.0, Unit::Scalar)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-qz")
        .unwrap();
        let gx = SensorActor::new(
            Arc::new(MockSensor::constant("imu/gx", 0.1, Unit::RadianPerSecond)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-gx")
        .unwrap();
        let gy = SensorActor::new(
            Arc::new(MockSensor::constant("imu/gy", 0.2, Unit::RadianPerSecond)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-gy")
        .unwrap();
        let gz = SensorActor::new(
            Arc::new(MockSensor::constant("imu/gz", 0.3, Unit::RadianPerSecond)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-gz")
        .unwrap();
        let ax = SensorActor::new(
            Arc::new(MockSensor::constant("imu/ax", 0.0, Unit::MetrePerSecondSquared)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-ax")
        .unwrap();
        let ay = SensorActor::new(
            Arc::new(MockSensor::constant("imu/ay", 0.0, Unit::MetrePerSecondSquared)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-ay")
        .unwrap();
        let az = SensorActor::new(
            Arc::new(MockSensor::constant("imu/az", 9.81, Unit::MetrePerSecondSquared)),
            SamplingPolicy::OnDemand,
        )
        .spawn(&sys, "imu-az")
        .unwrap();

        let agg = ImuAggregator {
            quat_w: qw,
            quat_x: qx,
            quat_y: qy,
            quat_z: qz,
            gyro_x: gx,
            gyro_y: gy,
            gyro_z: gz,
            accel_x: ax,
            accel_y: ay,
            accel_z: az,
            rate: LoopRate::new(Duration::from_millis(20)),
        };
        let agg_ref = agg.spawn(&sys, "imu-agg").unwrap();
        let mut rx = agg_ref.subscribe();

        let reading = tokio::time::timeout(Duration::from_millis(500), rx.recv())
            .await
            .expect("imu broadcast timed out")
            .expect("imu broadcast recv");
        // Identity quaternion stays identity.
        assert!((reading.orientation.w - 1.0).abs() < 1e-9);
        assert!((reading.angular_velocity.x - 0.1).abs() < 1e-9);
        assert!((reading.angular_velocity.y - 0.2).abs() < 1e-9);
        assert!((reading.angular_velocity.z - 0.3).abs() < 1e-9);
        assert!((reading.linear_acceleration.z - 9.81).abs() < 1e-9);

        sys.terminate().await;
    }
}
