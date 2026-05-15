//! `atomr-physical` — operate the physical layer from the command line.
//!
//! The `devices` / `sense` / `actuate` subcommands are scaffolded: each
//! prints the plan it would carry out and the crate APIs it would call,
//! pending a device registry to back the CLI. The `ros2 codecs`
//! subcommand is live — it inspects the real codec registry.

use anyhow::Result;
use atomr_physical_ros2::{unit_constraint, CodecRegistry, UnitConstraint};
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
    /// List the message-codec registry and the unit-compatibility table.
    Codecs,
    /// Spin the ROS2 bridge for a robot (requires the `rclrs` feature).
    Spin {
        /// The robot id to bridge onto a ROS2 node.
        robot: String,
    },
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let cli = Cli::parse();
    match cli.command {
        Cmd::Devices { kind } => {
            let filter = kind.as_deref().unwrap_or("all");
            println!("devices (kind filter: {filter})");
            println!("  (stub — enumerates `DeviceDescriptor`s once a device registry backs the CLI)");
        }
        Cmd::Sense { sensor } => {
            println!("sense {sensor}");
            println!(
                "  (stub — resolves the `SensorActor` for {sensor:?} and calls \
                 `SensorActor::sample`, printing the calibrated `Reading`)"
            );
        }
        Cmd::Actuate {
            actuator,
            mode,
            setpoint,
        } => {
            println!("actuate {actuator} mode={mode} setpoint={setpoint}");
            println!(
                "  (stub — builds a `Command`, runs it through the `ActuatorActor`'s \
                 `SafetyEnvelope`, and prints the `CommandAck`)"
            );
        }
        Cmd::Ros2 { op } => match op {
            Ros2Cmd::Plan { robot } => {
                println!("ros2 plan {robot}");
                println!(
                    "  (stub — builds the `Ros2Bridge` `Ros2Plan` for {robot:?} and prints \
                     each topic / service / action / parameter endpoint, once a device \
                     registry backs the CLI)"
                );
            }
            Ros2Cmd::Codecs => print_codecs(),
            Ros2Cmd::Spin { robot } => {
                println!("ros2 spin {robot}");
                println!(
                    "  (stub — calls `Ros2Bridge::run`; returns `PhysicalError::Ros2Bridge` \
                     unless built with `--features atomr-physical-ros2/rclrs` on a ROS 2 \
                     Jazzy host)"
                );
            }
        },
    }
    Ok(())
}

/// Print the codec registry and the `Unit` ↔ message-type table.
fn print_codecs() {
    let registry = CodecRegistry::builtin();
    let mut types: Vec<&str> = registry.registered_types().collect();
    types.sort_unstable();

    println!("registered codecs ({}):", registry.len());
    for message_type in &types {
        println!("  {message_type:<34}  {}", describe_units(message_type));
    }
    println!();
    println!(
        "the `rclrs` transport materialises each structured payload into a concrete\n\
         rosidl message at the wire; downstream crates add codecs via \
         `CodecRegistry::register`."
    );
}

/// A one-line description of a message type's unit constraint.
fn describe_units(message_type: &str) -> String {
    match unit_constraint(message_type) {
        UnitConstraint::Any => "any unit".to_string(),
        UnitConstraint::OneOf(units) => {
            let names: Vec<&str> = units.iter().map(|u| u.symbol()).collect();
            format!("units: {}", names.join(", "))
        }
        UnitConstraint::Unlisted => "no unit constraint listed".to_string(),
    }
}
