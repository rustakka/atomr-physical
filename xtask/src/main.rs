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
            println!("  1. cargo check --workspace --all-features");
            println!("  2. cargo test  --workspace");
            println!("  3. cargo publish -p <crate> --dry-run   (each crate, in dep order)");
            println!("  4. cargo doc   --workspace --no-deps");
        }
    }
    Ok(())
}
