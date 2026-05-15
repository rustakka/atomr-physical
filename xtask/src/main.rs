//! `xtask` — workspace tooling for atomr-physical.
//!
//! Invoke as `cargo xtask <cmd>`. The Cargo workspace runner alias
//! lives in `.cargo/config.toml`.

use std::process::{Command, ExitStatus, Stdio};

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "xtask", about = "atomr-physical workspace tooling")]
struct Cli {
    #[command(subcommand)]
    command: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Audit the workspace: `cargo fmt --check`, `cargo clippy
    /// --workspace --all-targets -- -D warnings`, and `cargo deny
    /// check` (if `cargo-deny` is installed).
    Audit {
        /// Skip `cargo deny`. Useful in CI before `cargo-deny` is on
        /// `PATH`.
        #[arg(long)]
        no_deny: bool,
    },
    /// Bump the workspace version. Delegates to `cargo set-version
    /// --workspace`, then updates `pyproject.toml` to match.
    Bump {
        /// The bump kind: `patch`, `minor`, or `major`. Mutually
        /// exclusive with `--exact`.
        #[arg(value_parser = ["patch", "minor", "major"])]
        kind: Option<String>,
        /// Set the workspace version to an exact value (e.g. `0.2.0`).
        #[arg(long, conflicts_with = "kind")]
        exact: Option<String>,
    },
    /// Print the pre-release pre-flight checklist.
    ReleaseChecklist,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    match cli.command {
        Cmd::Audit { no_deny } => audit(no_deny),
        Cmd::Bump { kind, exact } => bump(kind, exact),
        Cmd::ReleaseChecklist => {
            release_checklist();
            Ok(())
        }
    }
}

fn audit(no_deny: bool) -> Result<()> {
    println!("xtask audit");
    run_cargo(&["fmt", "--all", "--", "--check"]).context("cargo fmt --check failed")?;
    run_cargo(&["clippy", "--workspace", "--all-targets", "--", "-D", "warnings"])
        .context("cargo clippy failed")?;
    if !no_deny {
        match try_run_cargo(&["deny", "check"]) {
            Ok(status) if status.success() => {}
            Ok(status) => bail!("cargo deny check failed: {status}"),
            Err(_) => {
                println!("  cargo-deny not installed — skipping (`cargo install cargo-deny` to enable)");
            }
        }
    }
    println!("xtask audit OK");
    Ok(())
}

fn bump(kind: Option<String>, exact: Option<String>) -> Result<()> {
    let new_version: String = match (kind, exact) {
        (None, None) => bail!("xtask bump: pass either a kind (patch/minor/major) or --exact X.Y.Z"),
        (Some(k), None) => {
            // `cargo set-version --workspace --bump <kind>` requires the
            // `cargo-edit` toolchain — bail with a clear message if it
            // isn't installed.
            ensure_cargo_edit_installed()?;
            run_cargo(&["set-version", "--workspace", "--bump", &k])
                .context("cargo set-version --bump failed")?;
            read_workspace_version()?
        }
        (None, Some(v)) => {
            ensure_cargo_edit_installed()?;
            run_cargo(&["set-version", "--workspace", &v])
                .with_context(|| format!("cargo set-version --workspace {v} failed"))?;
            v
        }
        (Some(_), Some(_)) => unreachable!("clap enforces conflicts_with"),
    };
    sync_pyproject_version(&new_version).context("syncing pyproject.toml")?;
    println!("xtask bump → workspace + pyproject.toml at {new_version}");
    Ok(())
}

fn release_checklist() {
    println!("atomr-physical release pre-flight — see RELEASING.md");
    println!("  1. cargo check --workspace --all-features");
    println!("  2. cargo test  --workspace");
    println!("  3. cargo publish -p <crate> --dry-run   (each crate, in dep order)");
    println!("  4. cargo doc   --workspace --no-deps");
    println!("  5. cargo build -p atomr-physical-ros2 --features rclrs");
    println!("     (only meaningful in a ROS-sourced shell; needs the");
    println!("     rcl/rmw/msg .so's reachable via AMENT_PREFIX_PATH)");
}

fn run_cargo(args: &[&str]) -> Result<()> {
    let status = try_run_cargo(args)?;
    if !status.success() {
        bail!("cargo {} exited with {}", args.join(" "), status);
    }
    Ok(())
}

fn try_run_cargo(args: &[&str]) -> Result<ExitStatus> {
    let cargo = std::env::var("CARGO").unwrap_or_else(|_| "cargo".into());
    println!("  $ {} {}", cargo, args.join(" "));
    Command::new(cargo)
        .args(args)
        // pyo3 0.22 rejects Python >= 3.14 unless this is set; the
        // workaround lives in `memory/pyo3_python314.md`. Setting it
        // here means `cargo xtask audit` works on hosts with Python 3.14
        // even if the user hasn't exported it manually.
        .env("PYO3_USE_ABI3_FORWARD_COMPATIBILITY", "1")
        .stdin(Stdio::null())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit())
        .status()
        .with_context(|| format!("failed to spawn cargo {}", args.join(" ")))
}

fn ensure_cargo_edit_installed() -> Result<()> {
    // `cargo set-version --help` is a cheap probe that errors out
    // distinctively if the subcommand isn't installed.
    let status = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()))
        .args(["set-version", "--help"])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .context("probing for cargo-edit")?;
    if !status.success() {
        bail!(
            "cargo-edit is required for `xtask bump` — install it with \
             `cargo install cargo-edit`"
        );
    }
    Ok(())
}

fn read_workspace_version() -> Result<String> {
    let manifest = std::fs::read_to_string(workspace_root().join("Cargo.toml"))
        .context("reading workspace Cargo.toml")?;
    for line in manifest.lines() {
        let trimmed = line.trim();
        if let Some(rest) = trimmed.strip_prefix("version") {
            // Match `version = "X.Y.Z"` in `[workspace.package]`.
            if let Some(eq) = rest.find('=') {
                let value = rest[eq + 1..].trim();
                if let Some(v) = value.strip_prefix('"') {
                    if let Some(v) = v.strip_suffix('"') {
                        return Ok(v.to_string());
                    }
                }
            }
        }
    }
    bail!("could not parse workspace.package version from Cargo.toml")
}

fn sync_pyproject_version(new_version: &str) -> Result<()> {
    let path = workspace_root().join("pyproject.toml");
    let original = std::fs::read_to_string(&path).context("reading pyproject.toml")?;
    let mut out = String::with_capacity(original.len());
    let mut in_project = false;
    let mut bumped = false;
    for line in original.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with('[') {
            in_project = trimmed == "[project]";
        }
        if in_project && !bumped && trimmed.starts_with("version") && trimmed.contains('=') {
            // Preserve leading whitespace.
            let leading: String = line.chars().take_while(|c| c.is_whitespace()).collect::<String>();
            out.push_str(&leading);
            out.push_str(&format!("version = \"{new_version}\""));
            out.push('\n');
            bumped = true;
            continue;
        }
        out.push_str(line);
        out.push('\n');
    }
    if !bumped {
        bail!("could not find `version = \"…\"` under [project] in pyproject.toml");
    }
    // Preserve original trailing-newline shape: only write a trailing
    // newline if the input had one.
    if !original.ends_with('\n') {
        out.pop();
    }
    std::fs::write(&path, out).context("writing pyproject.toml")?;
    Ok(())
}

fn workspace_root() -> std::path::PathBuf {
    // CARGO_MANIFEST_DIR points at xtask/, so the workspace root is one
    // directory up.
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask Cargo manifest has a parent")
        .to_path_buf()
}
