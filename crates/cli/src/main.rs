//! `atomr-physical` — operate the physical layer from the command line.
//!
//! The CLI exercises the runtime end-to-end against an in-process device
//! registry seeded with the testkit mocks. It's the smallest demo of the
//! atomr-physical pipeline that doesn't require any hardware:
//!
//! * `devices`     — list everything in the registry, filterable by kind.
//! * `sense  <id>` — pull a calibrated reading from a sensor.
//! * `actuate <id> <setpoint> [--mode]` — clamp through a safety
//!   envelope and dispatch to an actuator.
//! * `ros2 plan <robot>` — print the per-device ROS2 topic map.
//! * `ros2 spin <robot>` — drive the bridge against a live ROS2 graph
//!   (requires the binary built with `--features rclrs`).
//!
//! The "registry" is a tiny in-process map seeded with one mock sensor
//! plus one mock actuator. Real deployments swap the registry for one
//! that talks to actual hardware drivers — the rest of the pipeline is
//! identical.

use std::collections::HashMap;
use std::sync::Arc;

use anyhow::{Context, Result};
use atomr_physical_actuation::{ActuatorActor, SafetyEnvelope};
use atomr_physical_core::{
    ActuatorId, Command, ControlMode, Device, DeviceDescriptor, DeviceKind, Quantity, Reading, RobotId,
    SensorId, Unit,
};
use atomr_physical_robotics::{Joint, RobotModel};
use atomr_physical_ros2::{Ros2Bridge, Ros2Direction, Ros2Endpoint};
use atomr_physical_sensing::{SamplingPolicy, SensorActor};
use atomr_physical_testkit::{MockActuator, MockSensor};
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(
    name = "atomr-physical",
    version,
    about = "Inspect devices, take readings, dispatch commands, and plan the ROS2 bridge."
)]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// List the devices registered with the physical layer.
    Devices {
        /// Filter to one device kind: `sensor`, `actuator`, or `composite`.
        #[arg(long)]
        kind: Option<String>,
    },
    /// Take a one-shot reading from a sensor.
    Sense {
        /// The sensor id to read from.
        sensor: String,
    },
    /// Dispatch a command to an actuator.
    Actuate {
        /// The actuator id to drive.
        actuator: String,
        /// The control mode: `position`, `velocity`, `effort`, or `duty`.
        #[arg(long, default_value = "position")]
        mode: String,
        /// The setpoint value.
        setpoint: f64,
        /// The unit the setpoint is expressed in (default: derived from the mode).
        #[arg(long)]
        unit: Option<String>,
    },
    /// ROS2 bridge operations.
    Ros2 {
        #[command(subcommand)]
        op: Ros2Cmd,
    },
}

#[derive(Subcommand)]
enum Ros2Cmd {
    /// Print the planned ROS2 topic graph for a robot.
    Plan {
        /// The robot id to plan a node graph for.
        robot: String,
    },
    /// Spin the ROS2 bridge for a robot (requires the `rclrs` feature).
    Spin {
        /// The robot id to bridge onto a ROS2 node.
        robot: String,
        /// How long to spin before shutting the bridge down (milliseconds).
        #[arg(long, default_value_t = 2000)]
        duration_ms: u64,
    },
}

/// In-process registry that backs the CLI. Real deployments replace the
/// driver constructors with hardware-backed ones.
struct DeviceRegistry {
    sensors: HashMap<SensorId, SensorActor>,
    actuators: HashMap<ActuatorId, ActuatorActor>,
    descriptors: Vec<DeviceDescriptor>,
    model: RobotModel,
    robot: RobotId,
}

impl DeviceRegistry {
    /// Seeds the registry with one mock sensor + one mock actuator
    /// arranged as a single-joint robot. The setup matches the README
    /// example so the CLI walks the same pipeline downstream code does.
    fn seeded() -> Self {
        let robot = RobotId::from("demo");

        let sensor_driver = Arc::new(MockSensor::constant("imu-temp", 21.5, Unit::Celsius));
        let sensor_descriptor = sensor_driver.descriptor().clone();
        let sensor = SensorActor::new(sensor_driver, SamplingPolicy::default_rate());

        let actuator_driver = Arc::new(MockActuator::new("joint-0"));
        let actuator_descriptor = actuator_driver.descriptor().clone();
        let actuator =
            ActuatorActor::new(actuator_driver).with_envelope(SafetyEnvelope::clamping(-1.57, 1.57));

        let mut model = RobotModel::new();
        model = model.with_joint(
            Joint::new(
                atomr_physical_core::JointId::from("j0"),
                "shoulder_pan",
                ActuatorId::from("joint-0"),
            )
            .with_feedback(SensorId::from("imu-temp")),
        );

        let mut sensors = HashMap::new();
        sensors.insert(sensor.id(), sensor);
        let mut actuators = HashMap::new();
        actuators.insert(actuator.id(), actuator);

        Self {
            sensors,
            actuators,
            descriptors: vec![sensor_descriptor, actuator_descriptor],
            model,
            robot,
        }
    }

    fn descriptors(&self, filter: Option<DeviceKind>) -> Vec<&DeviceDescriptor> {
        self.descriptors
            .iter()
            .filter(|d| filter.map_or(true, |k| d.kind == k))
            .collect()
    }

    fn sensor(&self, id: &SensorId) -> Option<&SensorActor> {
        self.sensors.get(id)
    }

    fn actuator(&self, id: &ActuatorId) -> Option<&ActuatorActor> {
        self.actuators.get(id)
    }

    fn build_bridge(&self, node_name: &str) -> Ros2Bridge {
        let mut bridge = Ros2Bridge::new(node_name, self.robot.clone());
        // Bind every sensor to a /<robot>/sensors/<id>/data topic and
        // every actuator to a /<robot>/actuators/<id>/cmd topic. Real
        // deployments override this with a TopicMap built from the
        // device registry's own naming convention.
        let robot = sanitize_ros_name(self.robot.as_str());
        for sensor_id in self.sensors.keys() {
            let id = sanitize_ros_name(sensor_id.as_str());
            bridge.topics_mut().bind_sensor(
                sensor_id.clone(),
                Ros2Endpoint::publish(format!("/{robot}/sensors/{id}/data"), "std_msgs/msg/Float64"),
            );
        }
        for actuator_id in self.actuators.keys() {
            let id = sanitize_ros_name(actuator_id.as_str());
            bridge.topics_mut().bind_actuator(
                actuator_id.clone(),
                Ros2Endpoint::subscribe(format!("/{robot}/actuators/{id}/cmd"), "std_msgs/msg/Float64"),
            );
        }
        bridge
    }
}

/// ROS 2 topic names allow `[A-Za-z][A-Za-z0-9_]*` per segment after a
/// leading `/`. Device ids may contain hyphens (`imu-temp`,
/// `joint-0`); we substitute `_` so the topic plan is valid input for
/// `rclrs::Node::create_dynamic_publisher`.
fn sanitize_ros_name(s: &str) -> String {
    s.chars().map(|c| if c == '-' { '_' } else { c }).collect()
}

fn parse_kind(s: &str) -> Result<DeviceKind> {
    Ok(match s {
        "sensor" => DeviceKind::Sensor,
        "actuator" => DeviceKind::Actuator,
        "composite" => DeviceKind::Composite,
        other => anyhow::bail!("unknown device kind: {other:?}"),
    })
}

fn parse_mode(s: &str) -> Result<ControlMode> {
    Ok(match s {
        "position" => ControlMode::Position,
        "velocity" => ControlMode::Velocity,
        "effort" => ControlMode::Effort,
        "duty" => ControlMode::Duty,
        other => anyhow::bail!("unknown control mode: {other:?}"),
    })
}

fn parse_unit(s: &str) -> Result<Unit> {
    Ok(match s {
        "" | "scalar" => Unit::Scalar,
        "m" | "metre" | "meter" => Unit::Metre,
        "m/s" => Unit::MetrePerSecond,
        "rad" | "radian" => Unit::Radian,
        "rad/s" => Unit::RadianPerSecond,
        "N" | "newton" => Unit::Newton,
        "Nm" | "N·m" | "newton_metre" => Unit::NewtonMetre,
        "C" | "celsius" => Unit::Celsius,
        "Pa" | "pascal" => Unit::Pascal,
        "V" | "volt" => Unit::Volt,
        "A" | "ampere" => Unit::Ampere,
        "%" | "percent" => Unit::Percent,
        other => anyhow::bail!("unknown unit: {other:?}"),
    })
}

fn default_unit_for_mode(mode: ControlMode) -> Unit {
    match mode {
        ControlMode::Position => Unit::Radian,
        ControlMode::Velocity => Unit::RadianPerSecond,
        ControlMode::Effort => Unit::NewtonMetre,
        ControlMode::Duty => Unit::Percent,
        _ => Unit::Scalar,
    }
}

fn print_descriptor(d: &DeviceDescriptor) {
    let kind = match d.kind {
        DeviceKind::Sensor => "sensor",
        DeviceKind::Actuator => "actuator",
        DeviceKind::Composite => "composite",
        _ => "unknown",
    };
    let caps: Vec<String> = d
        .capabilities
        .iter()
        .map(|c| format!("{}:{}", c.name, c.unit.symbol()))
        .collect();
    println!(
        "  {} [{}] model={} caps=[{}]",
        d.id,
        kind,
        d.model,
        caps.join(", ")
    );
}

fn print_reading(r: &Reading) {
    println!(
        "  {} = {} @ {}{}",
        r.sensor,
        r.quantity,
        r.timestamp_ms,
        r.frame
            .as_deref()
            .map(|f| format!(" frame={f}"))
            .unwrap_or_default()
    );
}

async fn cmd_devices(registry: &DeviceRegistry, kind_filter: Option<String>) -> Result<()> {
    let filter = kind_filter.as_deref().map(parse_kind).transpose()?;
    let descriptors = registry.descriptors(filter);
    println!(
        "devices ({} matching, filter={})",
        descriptors.len(),
        kind_filter.as_deref().unwrap_or("any")
    );
    for d in descriptors {
        print_descriptor(d);
    }
    Ok(())
}

async fn cmd_sense(registry: &DeviceRegistry, sensor: String) -> Result<()> {
    let id = SensorId::from(sensor.clone());
    let actor = registry
        .sensor(&id)
        .with_context(|| format!("unknown sensor: {sensor}"))?;
    let reading = actor.sample().await?;
    println!("sense {sensor}");
    print_reading(&reading);
    Ok(())
}

async fn cmd_actuate(
    registry: &DeviceRegistry,
    actuator: String,
    mode: String,
    setpoint: f64,
    unit: Option<String>,
) -> Result<()> {
    let id = ActuatorId::from(actuator.clone());
    let actor = registry
        .actuator(&id)
        .with_context(|| format!("unknown actuator: {actuator}"))?;
    let mode = parse_mode(&mode)?;
    let unit = match unit {
        Some(u) => parse_unit(&u)?,
        None => default_unit_for_mode(mode),
    };
    let cmd = Command::now(id.clone(), mode, Quantity::new(setpoint, unit));
    println!(
        "actuate {} mode={:?} setpoint={}{} envelope={:?}",
        actuator,
        mode,
        setpoint,
        unit.symbol(),
        actor.envelope()
    );
    let ack = actor.dispatch(cmd).await?;
    println!(
        "  ack accepted={} detail={} acked_ms={}",
        ack.accepted,
        ack.detail.as_deref().unwrap_or("-"),
        ack.acked_ms
    );
    Ok(())
}

async fn cmd_ros2_plan(registry: &DeviceRegistry, robot: String) -> Result<()> {
    if registry.robot != RobotId::from(robot.clone()) {
        anyhow::bail!("unknown robot: {robot} (registry holds {})", registry.robot);
    }
    let bridge = registry.build_bridge(&format!("{}_node", robot.replace('-', "_")));
    println!(
        "ros2 plan {robot} (node={:?}, {} endpoints)",
        bridge.node_name(),
        bridge.topics().len()
    );
    println!("  joints: {}", registry.model.joints.len());
    for (sensor_id, endpoint) in bridge.topics().sensor_bindings() {
        let direction = match endpoint.direction {
            Ros2Direction::Publish => "publish",
            Ros2Direction::Subscribe => "subscribe",
        };
        println!(
            "  sensor   {} -> topic={} type={} direction={}",
            sensor_id, endpoint.topic, endpoint.message_type, direction
        );
    }
    for (actuator_id, endpoint) in bridge.topics().actuator_bindings() {
        let direction = match endpoint.direction {
            Ros2Direction::Publish => "publish",
            Ros2Direction::Subscribe => "subscribe",
        };
        println!(
            "  actuator {} -> topic={} type={} direction={}",
            actuator_id, endpoint.topic, endpoint.message_type, direction
        );
    }
    Ok(())
}

async fn cmd_ros2_spin(registry: &DeviceRegistry, robot: String, duration_ms: u64) -> Result<()> {
    if registry.robot != RobotId::from(robot.clone()) {
        anyhow::bail!("unknown robot: {robot} (registry holds {})", registry.robot);
    }
    let bridge = registry.build_bridge(&format!("{}_node", robot.replace('-', "_")));
    println!(
        "ros2 spin {robot} (node={:?}, {} endpoints, duration={}ms)",
        bridge.node_name(),
        bridge.topics().len(),
        duration_ms
    );
    let handle = bridge.spin().await?;
    tokio::time::sleep(std::time::Duration::from_millis(duration_ms)).await;
    handle.shutdown().await?;
    println!("  bridge spun down cleanly.");
    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let cli = Cli::parse();
    let registry = DeviceRegistry::seeded();
    match cli.command {
        Cmd::Devices { kind } => cmd_devices(&registry, kind).await?,
        Cmd::Sense { sensor } => cmd_sense(&registry, sensor).await?,
        Cmd::Actuate {
            actuator,
            mode,
            setpoint,
            unit,
        } => cmd_actuate(&registry, actuator, mode, setpoint, unit).await?,
        Cmd::Ros2 { op } => match op {
            Ros2Cmd::Plan { robot } => cmd_ros2_plan(&registry, robot).await?,
            Ros2Cmd::Spin { robot, duration_ms } => cmd_ros2_spin(&registry, robot, duration_ms).await?,
        },
    }
    Ok(())
}
