use std::process::Command as Proc;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "xtask", about = "atomr-physical workspace tooling")]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Audit the workspace lint baseline.
    Audit,
    /// Bump the workspace version (delegates to `cargo set-version`).
    Bump {
        /// The bump kind: `patch`, `minor`, or `major`.
        kind: String,
    },
    /// Print the pre-release pre-flight checklist.
    ReleaseChecklist,
    /// Run the `rclrs`-gated ROS2 bridge integration tests.
    ///
    /// Requires a sourced ROS 2 Jazzy environment on the host. In CI this
    /// is the `workflow_dispatch`-only `rclrs-bridge` job.
    Ros2It,
}

fn main() -> anyhow::Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Cmd::Audit => {
            println!("xtask audit — workspace lint baseline (Phase 2)");
        }
        Cmd::Bump { kind } => {
            println!("xtask bump {kind} — delegates to `cargo set-version --workspace` (Phase 2)");
        }
        Cmd::ReleaseChecklist => {
            println!("atomr-physical release pre-flight — see RELEASING.md");
            // `--all-features` would pull `rclrs`, which fails on a
            // release host with no ROS 2 toolchain — exercise `full`.
            println!("  1. cargo check --workspace --features full");
            println!("  2. cargo test  --workspace");
            println!("  3. cargo publish -p <crate> --dry-run   (each crate, in dep order)");
            println!("  4. cargo doc   --workspace --no-deps");
            println!("  5. cargo xtask ros2-it   (on a ROS 2 Jazzy host — the rclrs bridge)");
        }
        Cmd::Ros2It => return run_ros2_it(),
    }
    Ok(())
}

/// Run the `rclrs`-gated ROS2 bridge integration tests.
fn run_ros2_it() -> anyhow::Result<()> {
    if std::env::var_os("ROS_DISTRO").is_none() {
        anyhow::bail!(
            "ROS_DISTRO is not set — `cargo xtask ros2-it` needs a sourced ROS 2 Jazzy \
             environment (`source /opt/ros/jazzy/setup.bash`). See docs/ros2-bridge.md §12."
        );
    }
    println!("xtask ros2-it — running the rclrs-gated ROS2 bridge integration tests");
    let status = Proc::new("cargo")
        .args([
            "test",
            "-p",
            "atomr-physical-ros2",
            "--features",
            "rclrs",
            "--test",
            "rclrs_integration",
        ])
        .status()?;
    if !status.success() {
        anyhow::bail!("rclrs integration tests failed");
    }
    Ok(())
}
