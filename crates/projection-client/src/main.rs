//! atomr-projection-client — remote Moonlight node companion.
//!
//! Discovers `_nvstream._tcp.local.` services advertised by
//! `atomr-physical-projection`, pairs against them over Sunshine's
//! self-signed HTTPS API, and execs `moonlight-embedded`.
//!
//! This binary is meant to ship on small ARM nodes (Raspberry Pi /
//! Jetson Nano) that act as remote screens for an atomr-physical
//! workstation. It is intentionally minimal: a single process, no
//! actor runtime, no persistent state — pair, stream, exit.

use anyhow::{anyhow, bail, Context, Result};
use atomr_physical_core::ClientId;
use clap::{Parser, Subcommand};
use mdns_sd::{ServiceDaemon, ServiceEvent, ServiceInfo};
use rand::Rng;
use regex::Regex;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::process::Command;
use tracing::{debug, error, info, warn};

/// mDNS service type used by Sunshine / NVIDIA GameStream for stream
/// discovery on the LAN. Servers spawned by `atomr-physical-projection`
/// advertise under this type with an `atomr-<short-id>` instance name.
const NVSTREAM_SERVICE_TYPE: &str = "_nvstream._tcp.local.";

/// Top-level CLI surface.
#[derive(Parser)]
#[command(
    name = "atomr-projection-client",
    version,
    about = "Discover atomr-physical Sunshine streams via mDNS, pair, then exec moonlight-embedded."
)]
struct Cli {
    /// Subcommand to run.
    #[command(subcommand)]
    cmd: Cmd,

    /// Increase verbosity (info → debug for this crate).
    #[arg(long, short, global = true)]
    verbose: bool,
}

/// Available subcommands.
#[derive(Subcommand)]
enum Cmd {
    /// Browse the LAN and print matching Sunshine services, then exit.
    Discover {
        /// Regex matched against the mDNS instance name (the segment
        /// before `._nvstream._tcp.local.`).
        #[arg(long, default_value = "atomr-.*")]
        service_filter: String,
        /// Seconds to spend collecting `ServiceResolved` events before
        /// printing what was found.
        #[arg(long, default_value_t = 5)]
        timeout_secs: u64,
    },
    /// Pair against the first matching service and exec
    /// `moonlight-embedded` against it.
    Run {
        /// Regex matched against the mDNS instance name.
        #[arg(long, default_value = "atomr-.*")]
        service_filter: String,
        /// Reuse a specific [`ClientId`]; defaults to a fresh
        /// `ClientId::new()` per invocation.
        #[arg(long)]
        client_id: Option<String>,
        /// Friendly hostname sent in the pairing payload. Defaults to
        /// `$HOSTNAME` or a literal fallback.
        #[arg(long)]
        hostname: Option<String>,
        /// Path to the `moonlight-embedded` binary to exec.
        #[arg(long, default_value = "moonlight-embedded")]
        moonlight_bin: PathBuf,
        /// Seconds to wait for a matching service to appear on the LAN.
        #[arg(long, default_value_t = 10)]
        discover_timeout_secs: u64,
        /// Print the generated PIN to stdout and wait for the operator
        /// to type it on the server before continuing.
        #[arg(long)]
        manual_pin: bool,
        /// Pair but don't actually exec `moonlight-embedded` (CI hook).
        #[arg(long)]
        dry_run: bool,
    },
}

/// A resolved nvstream advertisement we've decided is worth talking to.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiscoveredService {
    /// Full mDNS instance name, e.g.
    /// `"atomr-abc12345._nvstream._tcp.local."`.
    pub instance_name: String,
    /// First usable address (preferring IPv4) or the `.local.` hostname
    /// as a fallback if no addresses were resolved.
    pub host: String,
    /// TCP port the Sunshine HTTPS pairing API listens on, as taken from
    /// the SRV record.
    pub port: u16,
    /// TXT key/value pairs attached to the service record. May be empty
    /// if the underlying `mdns-sd` version exposes a different property
    /// surface than we expected.
    pub txt: HashMap<String, String>,
}

/// JSON body for `POST /api/pair`.
#[derive(Debug, Serialize)]
struct PairRequest<'a> {
    uuid: &'a str,
    name: &'a str,
}

/// JSON body for `POST /api/pin`.
#[derive(Debug, Serialize)]
struct PinRequest<'a> {
    pin: &'a str,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    init_tracing(cli.verbose);

    match cli.cmd {
        Cmd::Discover {
            service_filter,
            timeout_secs,
        } => cmd_discover(&service_filter, timeout_secs).await,
        Cmd::Run {
            service_filter,
            client_id,
            hostname,
            moonlight_bin,
            discover_timeout_secs,
            manual_pin,
            dry_run,
        } => {
            cmd_run(
                &service_filter,
                client_id,
                hostname,
                moonlight_bin,
                discover_timeout_secs,
                manual_pin,
                dry_run,
            )
            .await
        }
    }
}

/// Install `tracing_subscriber` honoring `RUST_LOG` plus the
/// `--verbose` flag (which forces `debug` for this crate).
fn init_tracing(verbose: bool) {
    // `mdns_sd` logs a noisy ERROR ("exit: failed to send response of
    // shutdown: sending on a closed channel") during its normal daemon
    // teardown path — dial it down so a clean exit looks clean.
    let default = if verbose {
        "info,atomr_projection_client=debug,mdns_sd=off"
    } else {
        "info,mdns_sd=off"
    };
    let env_filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new(default));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .with_target(false)
        .try_init();
}

/// Implementation of the `discover` subcommand.
async fn cmd_discover(service_filter: &str, timeout_secs: u64) -> Result<()> {
    let filter = Regex::new(service_filter)
        .with_context(|| format!("invalid --service-filter regex: {service_filter}"))?;

    let services = browse(&filter, Duration::from_secs(timeout_secs)).await?;
    if services.is_empty() {
        warn!("no matching services found within {timeout_secs}s");
        return Ok(());
    }

    for svc in &services {
        let txt = format_txt(&svc.txt);
        println!("{} {}:{} {}", svc.instance_name, svc.host, svc.port, txt);
    }
    Ok(())
}

/// Implementation of the `run` subcommand.
async fn cmd_run(
    service_filter: &str,
    client_id: Option<String>,
    hostname: Option<String>,
    moonlight_bin: PathBuf,
    discover_timeout_secs: u64,
    manual_pin: bool,
    dry_run: bool,
) -> Result<()> {
    let filter = Regex::new(service_filter)
        .with_context(|| format!("invalid --service-filter regex: {service_filter}"))?;
    let client_id: ClientId = client_id.map(ClientId::from).unwrap_or_default();
    let hostname = hostname.unwrap_or_else(|| {
        std::env::var("HOSTNAME").unwrap_or_else(|_| "atomr-projection-client".to_string())
    });

    let services = browse(&filter, Duration::from_secs(discover_timeout_secs)).await?;
    let service = services
        .into_iter()
        .next()
        .ok_or_else(|| anyhow!("no service matching `{service_filter}` resolved within {discover_timeout_secs}s"))?;

    let pin: String = {
        let n: u32 = rand::thread_rng().gen_range(0..10_000);
        format!("{:04}", n)
    };

    info!(
        client = %client_id,
        host = %service.host,
        port = service.port,
        pin = %pin,
        "pairing"
    );

    pair(&service, client_id.as_str(), &hostname, &pin, manual_pin).await?;

    if dry_run {
        info!("--dry-run set; skipping moonlight exec");
        return Ok(());
    }

    exec_moonlight(&moonlight_bin, &service.host).await
}

/// Browse mDNS for [`NVSTREAM_SERVICE_TYPE`] for `budget` time, keeping
/// only resolved services whose instance name matches `filter`.
async fn browse(filter: &Regex, budget: Duration) -> Result<Vec<DiscoveredService>> {
    let daemon = ServiceDaemon::new().context("failed to start mDNS daemon")?;
    let receiver = daemon
        .browse(NVSTREAM_SERVICE_TYPE)
        .with_context(|| format!("failed to browse {NVSTREAM_SERVICE_TYPE}"))?;

    let deadline = Instant::now() + budget;
    let mut found: HashMap<String, DiscoveredService> = HashMap::new();

    loop {
        let now = Instant::now();
        if now >= deadline {
            break;
        }
        let remaining = deadline - now;
        match tokio::time::timeout(remaining, receiver.recv_async()).await {
            Err(_elapsed) => break,
            Ok(Err(e)) => {
                debug!(error = %e, "mDNS receiver closed");
                break;
            }
            Ok(Ok(event)) => match event {
                ServiceEvent::ServiceResolved(info) => {
                    let instance = info.get_fullname().to_string();
                    if !filter.is_match(&instance) {
                        debug!(instance = %instance, "skipping non-matching service");
                        continue;
                    }
                    let svc = to_discovered(&info);
                    debug!(?svc, "resolved");
                    found.insert(instance, svc);
                }
                ServiceEvent::SearchStarted(t) => debug!(ty = %t, "search started"),
                ServiceEvent::ServiceFound(_, n) => debug!(name = %n, "service found"),
                ServiceEvent::ServiceRemoved(_, n) => debug!(name = %n, "service removed"),
                ServiceEvent::SearchStopped(_) => debug!("search stopped"),
            },
        }
    }

    // Stop the daemon politely; ignore errors during shutdown.
    if let Err(e) = daemon.shutdown() {
        debug!(error = ?e, "mDNS daemon shutdown returned error");
    }

    Ok(found.into_values().collect())
}

/// Convert a resolved [`ServiceInfo`] into our owned summary record.
///
/// Prefers IPv4 addresses for `host`, falls back to any address, then
/// finally to the `.local.` hostname embedded in the record.
fn to_discovered(info: &ServiceInfo) -> DiscoveredService {
    let port = info.get_port();
    let instance_name = info.get_fullname().to_string();

    // Address selection: IPv4 first, then anything, then hostname.
    let host = {
        let addrs = info.get_addresses();
        let v4 = addrs.iter().find(|ip| ip.is_ipv4());
        match v4 {
            Some(ip) => ip.to_string(),
            None => match addrs.iter().next() {
                Some(ip) => ip.to_string(),
                None => {
                    let hn = info.get_hostname().to_string();
                    // Trim trailing dot for cleaner CLI output.
                    hn.trim_end_matches('.').to_string()
                }
            },
        }
    };

    let txt = extract_txt(info);

    DiscoveredService {
        instance_name,
        host,
        port,
        txt,
    }
}

/// Best-effort TXT-property extraction.
///
/// The mdns-sd 0.11 surface exposes `get_properties() -> &TxtProperties`
/// with an iterator yielding `&TxtProperty`, each carrying `.key()` and
/// `.val_str()`. If a future minor version reshuffles this we'd rather
/// return an empty map than fail the whole pairing flow.
fn extract_txt(info: &ServiceInfo) -> HashMap<String, String> {
    let mut out = HashMap::new();
    for prop in info.get_properties().iter() {
        out.insert(prop.key().to_string(), prop.val_str().to_string());
    }
    out
}

/// Render TXT map as `k=v,k=v` for the discover output line.
fn format_txt(txt: &HashMap<String, String>) -> String {
    if txt.is_empty() {
        return "-".to_string();
    }
    let mut entries: Vec<String> = txt.iter().map(|(k, v)| format!("{k}={v}")).collect();
    entries.sort();
    entries.join(",")
}

/// Run the two-step Sunshine pairing handshake: `POST /api/pair`,
/// then `POST /api/pin` (after the operator types the PIN on the
/// server side, if `manual_pin` was requested).
async fn pair(
    service: &DiscoveredService,
    client_uuid: &str,
    hostname: &str,
    pin: &str,
    manual_pin: bool,
) -> Result<()> {
    let base = format!("https://{}:{}", service.host, service.port);
    let http = reqwest::Client::builder()
        .danger_accept_invalid_certs(true)
        .timeout(Duration::from_secs(10))
        .build()
        .context("failed to build reqwest client")?;

    // Step 1: announce the client.
    let pair_url = format!("{base}/api/pair");
    debug!(%pair_url, "POST /api/pair");
    let resp = http
        .post(&pair_url)
        .json(&PairRequest {
            uuid: client_uuid,
            name: hostname,
        })
        .send()
        .await
        .with_context(|| format!("POST {pair_url} failed"))?;
    let status = resp.status();
    let body = resp.text().await.unwrap_or_default();
    if !status.is_success() {
        bail!("pair endpoint returned {status}: {body}");
    }
    info!(%status, "announced client to server");

    if manual_pin {
        println!("=================================================");
        println!(" atomr-projection-client pairing PIN: {pin}");
        println!(" Type this PIN into the server's CLI, then press");
        println!(" <Enter> here to continue.");
        println!("=================================================");
        let mut stdin = BufReader::new(tokio::io::stdin());
        let mut line = String::new();
        let _ = stdin.read_line(&mut line).await;
    }

    // Step 2: submit the PIN.
    let pin_url = format!("{base}/api/pin");
    debug!(%pin_url, "POST /api/pin");
    let resp = http
        .post(&pin_url)
        .json(&PinRequest { pin })
        .send()
        .await
        .with_context(|| format!("POST {pin_url} failed"))?;
    let status = resp.status();
    let body = resp.text().await.unwrap_or_default();
    if !status.is_success() {
        bail!("pin endpoint returned {status}: {body}");
    }

    info!(%status, "paired successfully");
    Ok(())
}

/// Spawn `moonlight-embedded`, inherit stdio, await its exit, and
/// propagate a non-zero exit code by calling [`std::process::exit`].
async fn exec_moonlight(moonlight_bin: &PathBuf, host: &str) -> Result<()> {
    info!(?moonlight_bin, %host, "exec moonlight-embedded");
    let status = Command::new(moonlight_bin)
        .arg("-app")
        .arg("Desktop")
        .arg(host)
        .spawn()
        .with_context(|| format!("failed to spawn {}", moonlight_bin.display()))?
        .wait()
        .await
        .context("waiting for moonlight-embedded failed")?;

    if !status.success() {
        let code = status.code().unwrap_or(1);
        error!(%status, "moonlight-embedded exited non-zero");
        std::process::exit(code);
    }
    Ok(())
}
