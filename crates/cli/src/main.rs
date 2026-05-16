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
//! * `sdr {probe,info,rx,tx}` — talk to a HackRF One via the
//!   `atomr-physical-sdr` actor (requires `--features sdr`; SigMF
//!   capture-to-disk requires `--features sdr-sigmf`).
//!
//! The "registry" is a tiny in-process map seeded with one mock sensor
//! plus one mock actuator. Real deployments swap the registry for one
//! that talks to actual hardware drivers — the rest of the pipeline is
//! identical.

use std::collections::HashMap;
use std::sync::Arc;

use std::path::PathBuf;

use anyhow::{Context, Result};
use atomr_physical_actuation::{ActuatorActor, SafetyEnvelope};
use atomr_physical_core::{
    ActuatorId, ClientId, Command, ControlMode, Device, DeviceDescriptor, DeviceKind, Quantity,
    Reading, RobotId, SensorId, Unit,
};
use atomr_physical_projection::{ProjectionActor, ProjectionSpec};
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
    /// Projection (Sunshine/Moonlight) output operations.
    Project {
        #[command(subcommand)]
        op: ProjectCmd,
    },
    /// Software-Defined Radio operations (HackRF One). Only compiled
    /// when the binary is built with `--features sdr`.
    #[cfg(feature = "sdr")]
    Sdr {
        #[command(subcommand)]
        op: SdrCmd,
    },
}

#[derive(Subcommand)]
enum ProjectCmd {
    /// Boot a projection supervisor and request N stub projections.
    Demo {
        /// Path to the Sunshine binary (use `/bin/sleep` for offline demos).
        #[arg(long, default_value = "/bin/sleep")]
        sunshine_binary: PathBuf,
        /// How many projections to spin up.
        #[arg(long, default_value_t = 1)]
        count: u16,
        /// How long to keep the projections running before tearing down (ms).
        #[arg(long, default_value_t = 750)]
        hold_ms: u64,
        /// Force the offline pathway (skip vkms / mDNS / pairing shell-outs).
        #[arg(long, default_value_t = true)]
        offline: bool,
        /// mDNS host label prefix.
        #[arg(long, default_value = "atomr")]
        host_label: String,
    },
    /// Start a projection and pair a single client (offline by default).
    Pair {
        /// Path to the Sunshine binary.
        #[arg(long, default_value = "/bin/sleep")]
        sunshine_binary: PathBuf,
        /// The hostname displayed for the client in the pairing book.
        #[arg(long, default_value = "demo-client")]
        hostname: String,
        /// 4-digit PIN to submit. Generated if absent.
        #[arg(long)]
        pin: Option<String>,
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

#[cfg(feature = "sdr")]
#[derive(Subcommand)]
enum SdrCmd {
    /// List the serial numbers of every connected HackRF.
    Probe,
    /// Open the first available HackRF and print board / firmware info.
    Info,
    /// Receive IQ for a fixed duration, printing a summary at the end.
    /// With `--out` and the `sdr-sigmf` feature, also write a SigMF
    /// pair to disk.
    Rx {
        /// Centre frequency. Accepts SI suffixes (e.g. `100M`, `2.4G`,
        /// or plain `123456789`).
        #[arg(long)]
        centre: String,
        /// Sample rate. Accepts SI suffixes (e.g. `4M`, `2000000`).
        #[arg(long)]
        rate: String,
        /// LNA gain, in dB (multiple of 8, 0..=40).
        #[arg(long, default_value_t = 16)]
        gain_lna: u8,
        /// VGA / baseband gain, in dB (multiple of 2, 0..=62).
        #[arg(long, default_value_t = 20)]
        gain_vga: u8,
        /// Enable the +14 dB front-end RF amplifier.
        #[arg(long)]
        amp: bool,
        /// Enable the antenna-port bias-T (DC power to a powered LNA).
        #[arg(long)]
        bias_t: bool,
        /// How long to capture for (milliseconds).
        #[arg(long)]
        duration_ms: u64,
        /// Optional SigMF capture base path — requires `sdr-sigmf`.
        #[arg(long)]
        out: Option<PathBuf>,
    },
    /// Submit a TX burst from a raw `ci8_le` file. Returns the backend's
    /// `Unsupported` error today (rs-hackrf 0.4 is RX-only).
    Tx {
        /// Centre frequency. Accepts SI suffixes (see `rx --centre`).
        #[arg(long)]
        centre: String,
        /// Sample rate. Accepts SI suffixes (see `rx --rate`).
        #[arg(long)]
        rate: String,
        /// Path to a file of interleaved `ci8_le` samples to transmit.
        #[arg(long)]
        file: PathBuf,
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

async fn cmd_project_demo(
    sunshine_binary: PathBuf,
    count: u16,
    hold_ms: u64,
    offline: bool,
    host_label: String,
) -> Result<()> {
    let system = atomr_core::actor::ActorSystem::create("projection-demo", atomr_config::Config::reference())
        .await
        .map_err(|e| anyhow::anyhow!("actor system: {e:?}"))?;
    let actor_ref = ProjectionActor::new(sunshine_binary)
        .with_test_offline(offline)
        .with_mdns_host_label(host_label)
        .spawn(&system, "projection-demo")
        .map_err(|e| anyhow::anyhow!("spawn projection actor: {e:?}"))?;
    println!("project demo: starting {count} projection(s)");
    let mut handles = Vec::with_capacity(count as usize);
    for i in 0..count {
        let spec = ProjectionSpec::defaults();
        let handle = actor_ref
            .request_projection(spec)
            .await
            .with_context(|| format!("request projection {i}"))?;
        println!(
            "  [{i}] projection={} instance={} display={} http_port={} mdns={}",
            handle.projection_id, handle.instance_id, handle.display_id, handle.port_window.http_port(), handle.mdns_service
        );
        handles.push(handle);
    }
    tokio::time::sleep(std::time::Duration::from_millis(hold_ms)).await;
    let summaries = actor_ref.list_instances().await?;
    println!("project demo: {} live instance summaries", summaries.len());
    for s in &summaries {
        println!(
            "  instance={} pid={:?} running={} last_exit={:?} ports={:?}/{:?}",
            s.id, s.pid, s.running, s.last_exit_code, s.port_window.tcp, s.port_window.udp
        );
    }
    for h in &handles {
        actor_ref.stop_instance(h.instance_id.clone()).await?;
        println!("  stopped instance={}", h.instance_id);
    }
    system.terminate().await;
    Ok(())
}

async fn cmd_project_pair(
    sunshine_binary: PathBuf,
    hostname: String,
    pin: Option<String>,
) -> Result<()> {
    use rand::Rng;
    let system = atomr_core::actor::ActorSystem::create("projection-pair", atomr_config::Config::reference())
        .await
        .map_err(|e| anyhow::anyhow!("actor system: {e:?}"))?;
    let actor_ref = ProjectionActor::new(sunshine_binary)
        .with_test_offline(true)
        .spawn(&system, "projection-pair")
        .map_err(|e| anyhow::anyhow!("spawn projection actor: {e:?}"))?;
    let handle = actor_ref.request_projection(ProjectionSpec::defaults()).await?;
    let client = ClientId::new();
    let pin = pin.unwrap_or_else(|| format!("{:04}", rand::thread_rng().gen_range(0..10000)));
    println!("pair: instance={} client={} pin={}", handle.instance_id, client, pin);
    let ticket = actor_ref
        .pair_client(handle.instance_id.clone(), client.clone(), hostname.clone())
        .await?;
    println!("  ticket: salt_len={}", ticket.salt.len());
    actor_ref
        .submit_pin(handle.instance_id.clone(), client.clone(), hostname, pin)
        .await?;
    let pairings = actor_ref.known_pairings().await?;
    println!("  pairings now: {}", pairings.len());
    for p in &pairings {
        println!(
            "    client={} instance={} hostname={} paired_at_ms={}",
            p.client_id, p.instance, p.hostname, p.paired_at_ms
        );
    }
    actor_ref.stop_instance(handle.instance_id).await?;
    system.terminate().await;
    Ok(())
}

/// Parse an unsigned-integer Hz value with optional SI suffix (`k`,
/// `M`, `G`, case-insensitive). Plain integers pass through unchanged.
/// Examples: `100M` → 100_000_000, `4M` → 4_000_000, `1G` →
/// 1_000_000_000, `100k` → 100_000, `123` → 123.
#[cfg(feature = "sdr")]
fn parse_hz(s: &str) -> Result<u64> {
    let trimmed = s.trim();
    if trimmed.is_empty() {
        anyhow::bail!("empty frequency/rate value");
    }
    let last = trimmed.chars().last().unwrap();
    let (number_part, multiplier): (&str, u64) = match last {
        'k' | 'K' => (&trimmed[..trimmed.len() - 1], 1_000),
        'm' | 'M' => (&trimmed[..trimmed.len() - 1], 1_000_000),
        'g' | 'G' => (&trimmed[..trimmed.len() - 1], 1_000_000_000),
        '0'..='9' => (trimmed, 1),
        other => anyhow::bail!("unknown frequency suffix {other:?} in {s:?}"),
    };
    // Accept fractional inputs like `2.4G` by routing through f64 only
    // when the number portion has a `.`. Pure integers stay integer.
    if number_part.contains('.') {
        let n: f64 = number_part
            .parse()
            .map_err(|e| anyhow::anyhow!("bad number {number_part:?} in {s:?}: {e}"))?;
        let scaled = n * multiplier as f64;
        if !scaled.is_finite() || scaled < 0.0 {
            anyhow::bail!("frequency {s:?} is not a finite non-negative value");
        }
        Ok(scaled as u64)
    } else {
        let n: u64 = number_part
            .parse()
            .map_err(|e| anyhow::anyhow!("bad number {number_part:?} in {s:?}: {e}"))?;
        n.checked_mul(multiplier)
            .ok_or_else(|| anyhow::anyhow!("frequency {s:?} overflows u64"))
    }
}

#[cfg(feature = "sdr")]
async fn cmd_sdr_probe() -> Result<()> {
    match atomr_physical_sdr::HackRfDriver::probe() {
        Ok(serials) => {
            for s in serials {
                println!("{s}");
            }
            Ok(())
        }
        Err(e) => {
            eprintln!("sdr probe failed: {e}");
            anyhow::bail!("sdr probe failed: {e}");
        }
    }
}

#[cfg(feature = "sdr")]
async fn cmd_sdr_info() -> Result<()> {
    let driver = atomr_physical_sdr::HackRfDriver::open_first()
        .map_err(|e| anyhow::anyhow!("open hackrf: {e}"))?;
    let info = driver
        .info()
        .await
        .map_err(|e| anyhow::anyhow!("hackrf info: {e}"))?;
    println!(
        "board_id      = {} ({})",
        info.board_id,
        rs_hackrf::transport::board_id_name(info.board_id)
    );
    println!("version       = {}", info.version);
    println!("serial        = {}", info.serial);
    println!("usb_api_ver   = 0x{:04x}", info.usb_api_version);
    Ok(())
}

#[cfg(feature = "sdr")]
async fn cmd_sdr_rx(
    centre: String,
    rate: String,
    gain_lna: u8,
    gain_vga: u8,
    amp: bool,
    bias_t: bool,
    duration_ms: u64,
    out: Option<PathBuf>,
) -> Result<()> {
    use std::sync::Arc;
    use std::time::{Duration, Instant};

    if out.is_some() && !cfg!(feature = "sdr-sigmf") {
        anyhow::bail!("`--out` requires the `sdr-sigmf` feature — rebuild with --features sdr-sigmf");
    }

    let centre_hz = parse_hz(&centre)?;
    let rate_hz_u64 = parse_hz(&rate)?;
    let rate_hz: u32 = rate_hz_u64
        .try_into()
        .map_err(|_| anyhow::anyhow!("sample rate {rate_hz_u64} exceeds u32 range"))?;

    let params = atomr_physical_sdr::SdrParams::default_rx()
        .with_centre_hz(centre_hz)
        .with_sample_rate_hz(rate_hz)
        .with_lna_gain_db(gain_lna)
        .with_vga_gain_db(gain_vga)
        .with_amp_enable(amp)
        .with_antenna_port_pwr(bias_t);

    let driver = atomr_physical_sdr::HackRfDriver::open_first()
        .map_err(|e| anyhow::anyhow!("open hackrf: {e}"))?;
    let driver: Arc<dyn atomr_physical_sdr::SdrBackend> = Arc::new(driver);

    let system = atomr_core::actor::ActorSystem::create("sdr-cli", atomr_config::Config::reference())
        .await
        .map_err(|e| anyhow::anyhow!("actor system: {e:?}"))?;
    let actor_ref = atomr_physical_sdr::SdrActor::new(driver)
        .with_params(params.clone())
        .auto_start_rx(true)
        .spawn(&system, "sdr-cli")
        .map_err(|e| anyhow::anyhow!("spawn sdr actor: {e:?}"))?;

    println!(
        "sdr rx: centre={} Hz rate={} Hz lna={} vga={} amp={} bias_t={} duration={}ms{}",
        params.centre_hz,
        params.sample_rate_hz,
        params.lna_gain_db,
        params.vga_gain_db,
        params.amp_enable,
        params.antenna_port_pwr,
        duration_ms,
        out.as_ref()
            .map(|p| format!(" out={}", p.display()))
            .unwrap_or_default(),
    );

    let mut rx = actor_ref.subscribe();

    // Optional SigMF persistence — only compiled when sdr-sigmf is on.
    #[cfg(feature = "sdr-sigmf")]
    let sigmf_task: Option<tokio::task::JoinHandle<atomr_physical_sdr::SdrResult<atomr_physical_sdr::SigmfWriter>>> =
        if let Some(path) = out.clone() {
            let writer_rx = actor_ref.subscribe();
            let writer = atomr_physical_sdr::SigmfWriter::open(
                atomr_physical_sdr::PersistConfig::at(path),
            )
            .await
            .map_err(|e| anyhow::anyhow!("open sigmf writer: {e}"))?;
            Some(tokio::spawn(atomr_physical_sdr::persist_until_eos(
                writer_rx, writer,
            )))
        } else {
            None
        };

    let start = Instant::now();
    let deadline = start + Duration::from_millis(duration_ms);
    let mut chunks: u64 = 0;
    let mut sample_pairs: u64 = 0;
    let mut bytes: u64 = 0;
    loop {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        let remaining = deadline - now;
        match tokio::time::timeout(remaining, rx.recv()).await {
            Ok(Ok(chunk)) => {
                chunks += 1;
                sample_pairs += chunk.len_samples() as u64;
                bytes += chunk.len_bytes() as u64;
            }
            Ok(Err(tokio::sync::broadcast::error::RecvError::Lagged(n))) => {
                tracing::warn!(missed = n, "sdr rx: broadcast lagged");
                continue;
            }
            Ok(Err(tokio::sync::broadcast::error::RecvError::Closed)) => break,
            Err(_elapsed) => break,
        }
    }
    let elapsed = start.elapsed();

    // Stop RX before pulling the actor down.
    if let Err(e) = actor_ref.stop_rx().await {
        tracing::warn!(?e, "stop_rx returned error; tearing actor system down anyway");
    }

    #[cfg(feature = "sdr-sigmf")]
    {
        if let Some(task) = sigmf_task {
            match task.await {
                Ok(Ok(writer)) => {
                    println!(
                        "sigmf: wrote {} bytes to {}",
                        writer.bytes_written(),
                        writer.data_path().display(),
                    );
                }
                Ok(Err(e)) => {
                    tracing::error!(?e, "sigmf writer reported error");
                }
                Err(e) => {
                    tracing::error!(?e, "sigmf writer task panicked");
                }
            }
        }
    }

    system.terminate().await;

    let mib = bytes as f64 / (1024.0 * 1024.0);
    println!(
        "received {chunks} chunks ({sample_pairs} sample-pairs, {mib:.2} MiB) in {:.3}s",
        elapsed.as_secs_f64(),
    );
    Ok(())
}

#[cfg(feature = "sdr")]
async fn cmd_sdr_tx(centre: String, rate: String, file: PathBuf) -> Result<()> {
    use std::sync::Arc;

    let centre_hz = parse_hz(&centre)?;
    let rate_hz_u64 = parse_hz(&rate)?;
    let rate_hz: u32 = rate_hz_u64
        .try_into()
        .map_err(|_| anyhow::anyhow!("sample rate {rate_hz_u64} exceeds u32 range"))?;

    let bytes = std::fs::read(&file)
        .with_context(|| format!("read tx samples from {}", file.display()))?;
    // `i8` and `u8` share their byte layout — cast the slice in place.
    let samples: Vec<i8> = bytes.iter().map(|&b| b as i8).collect();
    let samples_arc: Arc<[i8]> = Arc::from(samples);

    let params = atomr_physical_sdr::SdrParams::default_rx()
        .with_centre_hz(centre_hz)
        .with_sample_rate_hz(rate_hz);

    let driver = atomr_physical_sdr::HackRfDriver::open_first()
        .map_err(|e| anyhow::anyhow!("open hackrf: {e}"))?;
    let driver: Arc<dyn atomr_physical_sdr::SdrBackend> = Arc::new(driver);

    let system = atomr_core::actor::ActorSystem::create("sdr-cli", atomr_config::Config::reference())
        .await
        .map_err(|e| anyhow::anyhow!("actor system: {e:?}"))?;
    let actor_ref = atomr_physical_sdr::SdrActor::new(driver)
        .with_params(params)
        .spawn(&system, "sdr-cli")
        .map_err(|e| anyhow::anyhow!("spawn sdr actor: {e:?}"))?;

    let result = actor_ref.transmit(samples_arc).await;
    system.terminate().await;
    match result {
        Ok(()) => {
            println!("sdr tx: submitted {} bytes", bytes.len());
            Ok(())
        }
        Err(e) => {
            eprintln!("sdr tx: {e}");
            std::process::exit(1);
        }
    }
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
        Cmd::Project { op } => match op {
            ProjectCmd::Demo {
                sunshine_binary,
                count,
                hold_ms,
                offline,
                host_label,
            } => cmd_project_demo(sunshine_binary, count, hold_ms, offline, host_label).await?,
            ProjectCmd::Pair {
                sunshine_binary,
                hostname,
                pin,
            } => cmd_project_pair(sunshine_binary, hostname, pin).await?,
        },
        #[cfg(feature = "sdr")]
        Cmd::Sdr { op } => match op {
            SdrCmd::Probe => cmd_sdr_probe().await?,
            SdrCmd::Info => cmd_sdr_info().await?,
            SdrCmd::Rx {
                centre,
                rate,
                gain_lna,
                gain_vga,
                amp,
                bias_t,
                duration_ms,
                out,
            } => {
                cmd_sdr_rx(
                    centre, rate, gain_lna, gain_vga, amp, bias_t, duration_ms, out,
                )
                .await?
            }
            SdrCmd::Tx { centre, rate, file } => cmd_sdr_tx(centre, rate, file).await?,
        },
    }
    Ok(())
}
