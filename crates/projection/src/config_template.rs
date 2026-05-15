//! Sunshine ini config rendering and on-disk placement.
//!
//! Each spawned Sunshine instance is fed a dedicated ini file that
//! pins it to a specific virtual display, encoder, bitrate, and port
//! window. This module is the pure layer that produces the file text
//! and decides where on disk it lives — it does not spawn Sunshine
//! itself; that is the job of [`crate::sunshine`].
//!
//! Render output is intentionally minimal: only the keys the supervisor
//! actually drives end up in the file. Operators can diff the rendered
//! file against the running config to confirm that the supervisor's
//! view matches reality.

use atomr_physical_core::{PhysicalError, Result, SunshineInstanceId};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use tracing::{debug, warn};

use crate::ports::PortWindow;

/// Default minimum log-level bitrate, in kbps.
const DEFAULT_MIN_LOG_BITRATE_KBPS: u32 = 2_000;

/// Default target bitrate, in kbps.
const DEFAULT_BITRATE_KBPS: u32 = 10_000;

/// Default ceiling bitrate, in kbps.
const DEFAULT_MAX_BITRATE_KBPS: u32 = 20_000;

/// Default frame rate.
const DEFAULT_FPS: u32 = 30;

/// Default resolution: 1080p.
const DEFAULT_RESOLUTION: (u32, u32) = (1920, 1080);

/// Subdirectory under the runtime root that holds projection state.
const RUNTIME_SUBDIR: &str = "atomr/projection";

/// Parameters for rendering a Sunshine ini config file.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SunshineConfigParams {
    /// The supervisor's id for this Sunshine instance.
    pub instance: SunshineInstanceId,
    /// The DRM device backing the virtual display (e.g.
    /// `/dev/dri/card0`).
    pub display_device: PathBuf,
    /// The connector / output name Sunshine should grab (e.g.
    /// `Virtual-1`).
    pub display_name: String,
    /// The reserved port window for this instance.
    pub port_window: PortWindow,
    /// One of `software`, `nvenc`, `vaapi`, `quicksync`.
    pub encoder: String,
    /// Target bitrate, in kbps.
    pub bitrate_kbps: u32,
    /// Bitrate below which Sunshine logs a quality warning.
    pub min_log_bitrate_kbps: u32,
    /// Hard upper bound on bitrate, in kbps.
    pub max_bitrate_kbps: u32,
    /// Stream frame rate.
    pub fps: u32,
    /// `(width, height)` of the encoded stream.
    pub resolution: (u32, u32),
    /// Path to the TLS private key Sunshine serves over HTTPS.
    pub pkey_path: PathBuf,
    /// Path to the TLS certificate Sunshine serves over HTTPS.
    pub cert_path: PathBuf,
}

impl SunshineConfigParams {
    /// A safe default for a single 1080p30 software-encoded stream.
    ///
    /// `pkey_path` and `cert_path` are left as `pkey.pem` / `cert.pem`
    /// relative paths — callers that need stable absolute locations
    /// should overwrite them before rendering.
    pub fn defaults_for(
        instance: SunshineInstanceId,
        port_window: PortWindow,
        display_device: PathBuf,
    ) -> Self {
        Self {
            instance,
            display_device,
            display_name: "Virtual-1".to_string(),
            port_window,
            encoder: "software".to_string(),
            bitrate_kbps: DEFAULT_BITRATE_KBPS,
            min_log_bitrate_kbps: DEFAULT_MIN_LOG_BITRATE_KBPS,
            max_bitrate_kbps: DEFAULT_MAX_BITRATE_KBPS,
            fps: DEFAULT_FPS,
            resolution: DEFAULT_RESOLUTION,
            pkey_path: PathBuf::from("pkey.pem"),
            cert_path: PathBuf::from("cert.pem"),
        }
    }
}

/// Render the params into a Sunshine ini string.
///
/// The emitted file uses Sunshine's `key = value` syntax. A short
/// banner naming the instance is included at the top so operators can
/// diff against the running config.
pub fn render_config(p: &SunshineConfigParams) -> String {
    let mut out = String::with_capacity(512);
    out.push_str("# atomr-physical-projection\n");
    out.push_str(&format!("# instance = {}\n", p.instance.as_str()));
    out.push_str("# rendered config; edits will be overwritten on respawn\n");
    out.push_str(&format!("output_name = {}\n", p.display_name));
    out.push_str(&format!(
        "adapter_name = {}\n",
        p.display_device.display()
    ));
    out.push_str(&format!("port = {}\n", p.port_window.tcp[1]));
    out.push_str("min_log_level = 2\n");
    out.push_str(&format!("encoder = {}\n", p.encoder));
    out.push_str(&format!("bitrate = {}\n", p.bitrate_kbps));
    out.push_str(&format!("min_log_bitrate = {}\n", p.min_log_bitrate_kbps));
    out.push_str(&format!("max_bitrate = {}\n", p.max_bitrate_kbps));
    out.push_str(&format!("fps = {}\n", p.fps));
    out.push_str(&format!("width = {}\n", p.resolution.0));
    out.push_str(&format!("height = {}\n", p.resolution.1));
    out.push_str(&format!("pkey = {}\n", p.pkey_path.display()));
    out.push_str(&format!("cert = {}\n", p.cert_path.display()));
    out
}

/// Resolve the runtime config directory for projection instances.
///
/// Honours `$XDG_RUNTIME_DIR`, falling back to `std::env::temp_dir()`.
/// The returned path is guaranteed to exist (created if necessary).
pub fn runtime_config_dir() -> Result<PathBuf> {
    let root = match std::env::var_os("XDG_RUNTIME_DIR") {
        Some(v) if !v.is_empty() => PathBuf::from(v),
        _ => {
            debug!("XDG_RUNTIME_DIR not set; falling back to std::env::temp_dir()");
            std::env::temp_dir()
        }
    };
    let dir = root.join(RUNTIME_SUBDIR);
    std::fs::create_dir_all(&dir).map_err(|e| {
        warn!(path = %dir.display(), error = %e, "failed to create projection config dir");
        PhysicalError::Fault(format!(
            "projection config dir {}: {e}",
            dir.display()
        ))
    })?;
    Ok(dir)
}

/// Write a rendered Sunshine config to a per-instance `TempDir`.
///
/// Returns the absolute path to the `.conf` file plus the
/// [`tempfile::TempDir`] guard — drop it to clean up. The `TempDir`
/// parents at [`runtime_config_dir`].
pub fn write_instance_config(
    instance: &SunshineInstanceId,
    contents: &str,
) -> Result<(PathBuf, tempfile::TempDir)> {
    let parent = runtime_config_dir()?;
    let dir = tempfile::Builder::new()
        .prefix(instance.as_str())
        .tempdir_in(&parent)
        .map_err(|e| {
            warn!(parent = %parent.display(), error = %e, "tempdir creation failed");
            PhysicalError::Fault(format!(
                "tempdir for instance {} in {}: {e}",
                instance.as_str(),
                parent.display()
            ))
        })?;
    let path = dir.path().join("sunshine.conf");
    std::fs::write(&path, contents).map_err(|e| {
        warn!(path = %path.display(), error = %e, "failed to write sunshine.conf");
        PhysicalError::Fault(format!(
            "write sunshine.conf at {}: {e}",
            path.display()
        ))
    })?;
    debug!(
        instance = instance.as_str(),
        path = %path.display(),
        "wrote sunshine config"
    );
    Ok((path, dir))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture_params() -> SunshineConfigParams {
        SunshineConfigParams {
            instance: SunshineInstanceId::from("sun-test-001"),
            display_device: PathBuf::from("/dev/dri/card0"),
            display_name: "Virtual-7".to_string(),
            port_window: PortWindow::at_offset(200),
            encoder: "vaapi".to_string(),
            bitrate_kbps: 12_000,
            min_log_bitrate_kbps: 3_000,
            max_bitrate_kbps: 24_000,
            fps: 60,
            resolution: (2560, 1440),
            pkey_path: PathBuf::from("/etc/atomr/pkey.pem"),
            cert_path: PathBuf::from("/etc/atomr/cert.pem"),
        }
    }

    #[test]
    fn render_config_emits_expected_keys() {
        let s = render_config(&fixture_params());
        assert!(s.contains("output_name = Virtual-7"));
        assert!(s.contains("adapter_name = /dev/dri/card0"));
        assert!(s.contains("port = 48189"));
        assert!(s.contains("min_log_level = 2"));
        assert!(s.contains("encoder = vaapi"));
        assert!(s.contains("bitrate = 12000"));
        assert!(s.contains("min_log_bitrate = 3000"));
        assert!(s.contains("max_bitrate = 24000"));
        assert!(s.contains("fps = 60"));
        assert!(s.contains("width = 2560"));
        assert!(s.contains("height = 1440"));
        assert!(s.contains("pkey = /etc/atomr/pkey.pem"));
        assert!(s.contains("cert = /etc/atomr/cert.pem"));
    }

    #[test]
    fn render_config_includes_instance_banner() {
        let s = render_config(&fixture_params());
        let head = s.lines().next().unwrap();
        assert!(head.starts_with("# atomr-physical-projection"));
        assert!(s.contains("instance = sun-test-001"));
    }

    #[test]
    fn defaults_for_uses_port_window() {
        let window = PortWindow::at_offset(300);
        let p = SunshineConfigParams::defaults_for(
            SunshineInstanceId::from("sun-def"),
            window,
            PathBuf::from("/dev/dri/card0"),
        );
        let s = render_config(&p);
        let expected = format!("port = {}", window.tcp[1]);
        assert!(
            s.contains(&expected),
            "expected `{expected}` in rendered config:\n{s}"
        );
        assert_eq!(p.fps, DEFAULT_FPS);
        assert_eq!(p.bitrate_kbps, DEFAULT_BITRATE_KBPS);
    }

    #[test]
    fn runtime_config_dir_creates_path() {
        let a = runtime_config_dir().expect("first call");
        let b = runtime_config_dir().expect("second call");
        assert_eq!(a, b);
        assert!(a.exists(), "runtime config dir was not created: {a:?}");
        assert!(a.ends_with(RUNTIME_SUBDIR));
    }

    #[test]
    fn write_instance_config_creates_file() {
        let id = SunshineInstanceId::from("sun-write-test");
        let body = "# unit test\nport = 47989\n";
        let (path, _guard) = write_instance_config(&id, body).expect("write");
        assert!(path.is_absolute(), "path should be absolute: {path:?}");
        assert_eq!(path.file_name().unwrap(), "sunshine.conf");
        let read_back = std::fs::read_to_string(&path).expect("read back");
        assert_eq!(read_back, body);
    }
}
