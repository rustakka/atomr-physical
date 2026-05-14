//! `atomr-physical` — operate the physical layer from the command line.
//!
//! The subcommands are scaffolded: each prints the plan it would carry
//! out and the crate APIs it would call. They are wired to real device
//! registries and the ROS2 bridge as the corresponding subsystems land
//! (see the per-command notes and `docs/architecture.md`).

use anyhow::Result;
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
                    "  (stub — builds the `Ros2Bridge` `TopicMap` for {robot:?} and prints \
                     each sensor/actuator endpoint)"
                );
            }
            Ros2Cmd::Spin { robot } => {
                println!("ros2 spin {robot}");
                println!(
                    "  (stub — calls `Ros2Bridge::spin`; returns `PhysicalError::Ros2Bridge` \
                     unless built with `--features atomr-physical-ros2/rclrs`)"
                );
            }
        },
    }
    Ok(())
}
