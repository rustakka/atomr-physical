//! Headless virtual-display lifecycle backed by the Linux **vkms** DRM
//! driver.
//!
//! [`VkmsDisplayManager`] is a plain async helper — not an actor. The
//! parent [`ProjectionActor`](crate::ProjectionActor) owns it as a
//! field and drives create / destroy under its own mailbox, so writes
//! are already serialised; this module provides the kernel-module
//! probe, DRM card discovery, and `xrandr` shell-outs that back each
//! call.
//!
//! Construct with [`VkmsDisplayManager::new`] in production or
//! [`VkmsDisplayManager::offline`] in tests — the offline manager
//! bypasses every shell-out and keeps state purely in the `active`
//! map, which is what the integration tests against a stub Sunshine
//! binary depend on.

use atomr_physical_core::{DisplayId, PhysicalError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use tokio::process::Command;
use tracing::{debug, info, warn};

/// Synthetic DRM card path returned by [`VkmsDisplayManager::offline`].
const OFFLINE_CARD: &str = "/dev/dri/card-test";

/// `/sys` path the module probe inspects to decide whether vkms is
/// already loaded.
const VKMS_SYS_PATH: &str = "/sys/module/vkms";

/// `/sys` root walked to discover the DRM card backing a connector.
const DRM_SYS_ROOT: &str = "/sys/class/drm";

/// Mode + connector definition for a single virtual display.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisplaySpec {
    /// `(width, height)` in pixels.
    pub resolution: (u32, u32),
    /// Refresh rate in Hz.
    pub refresh_hz: u32,
    /// Connector name to use, e.g. "Virtual-1" / "Virtual-2".
    pub connector: String,
}

impl DisplaySpec {
    /// Build a spec with explicit resolution, refresh, and connector
    /// name.
    pub fn new(width: u32, height: u32, refresh_hz: u32, connector: impl Into<String>) -> Self {
        Self {
            resolution: (width, height),
            refresh_hz,
            connector: connector.into(),
        }
    }

    /// 1920x1080 at 30 Hz on connector "Virtual-1" — the default
    /// projection profile used by the reference Sunshine config.
    pub fn hd_30() -> Self {
        Self::new(1920, 1080, 30, "Virtual-1")
    }
}

/// A live virtual display owned by a [`VkmsDisplayManager`].
///
/// The handle carries everything the Sunshine config emitter needs:
/// the connector name from the [`DisplaySpec`] and the DRM card path
/// discovered for it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DisplayHandle {
    /// Id minted at creation time. Stable for the lifetime of the
    /// display.
    pub id: DisplayId,
    /// Spec the display was created from.
    pub spec: DisplaySpec,
    /// DRM card device path discovered for this connector, e.g.
    /// `/dev/dri/card0`.
    pub drm_card: PathBuf,
}

/// Manages the lifecycle of vkms-backed virtual displays.
///
/// One per [`ProjectionActor`](crate::ProjectionActor). The manager
/// keeps an in-memory map of active displays and shells out to
/// `modprobe` / `xrandr` for kernel module + mode setup. Set
/// `allow_modprobe` to `false` on hosts where vkms is preloaded via
/// `/etc/modules-load.d/vkms.conf`.
pub struct VkmsDisplayManager {
    active: HashMap<DisplayId, DisplayHandle>,
    kernel_module_loaded: AtomicBool,
    allow_modprobe: bool,
    /// When set, skip every shell-out — tests inject this to operate
    /// purely in-memory.
    test_offline: bool,
}

impl VkmsDisplayManager {
    /// Build a manager that talks to the real kernel + xrandr.
    ///
    /// `allow_modprobe` decides whether
    /// [`ensure_module_loaded`](Self::ensure_module_loaded) is
    /// permitted to load vkms automatically, or whether it must
    /// already be present.
    pub fn new(allow_modprobe: bool) -> Self {
        Self {
            active: HashMap::new(),
            kernel_module_loaded: AtomicBool::new(false),
            allow_modprobe,
            test_offline: false,
        }
    }

    /// Construct an offline manager: bypasses every shell-out. Use
    /// this in unit tests and from the integration tests that
    /// exercise the projection actor against a stub Sunshine binary.
    pub fn offline() -> Self {
        Self {
            active: HashMap::new(),
            kernel_module_loaded: AtomicBool::new(true),
            allow_modprobe: false,
            test_offline: true,
        }
    }

    /// Number of displays currently held open.
    pub fn active_count(&self) -> usize {
        self.active.len()
    }

    /// Whether the manager is allowed to `modprobe vkms`.
    pub fn allow_modprobe(&self) -> bool {
        self.allow_modprobe
    }

    /// Probe `/sys/module/vkms`; if absent and `allow_modprobe` is
    /// true, shell out to `modprobe vkms` via
    /// [`tokio::process::Command`]. On `test_offline = true` this
    /// returns Ok without touching anything.
    ///
    /// The result is memoised in an `AtomicBool`, so repeat calls on
    /// the hot path are a single relaxed load.
    pub async fn ensure_module_loaded(&self) -> Result<()> {
        if self.kernel_module_loaded.load(Ordering::Relaxed) {
            return Ok(());
        }
        if self.test_offline {
            self.kernel_module_loaded.store(true, Ordering::Relaxed);
            return Ok(());
        }

        if Path::new(VKMS_SYS_PATH).exists() {
            debug!(path = VKMS_SYS_PATH, "vkms already loaded");
            self.kernel_module_loaded.store(true, Ordering::Relaxed);
            return Ok(());
        }

        if !self.allow_modprobe {
            return Err(PhysicalError::KernelModule {
                module: "vkms",
                reason: "module not loaded and auto-modprobe disabled; \
                         enable on-boot loading via /etc/modules-load.d/vkms.conf"
                    .to_string(),
            });
        }

        info!("loading vkms kernel module via modprobe");
        let output = Command::new("modprobe")
            .arg("vkms")
            .output()
            .await
            .map_err(|e| PhysicalError::KernelModule {
                module: "vkms",
                reason: format!(
                    "failed to spawn modprobe: {e}; \
                     ensure the kmod package is installed and try \
                     /etc/modules-load.d/vkms.conf"
                ),
            })?;

        if !output.status.success() {
            return Err(PhysicalError::KernelModule {
                module: "vkms",
                reason: format!(
                    "modprobe vkms failed: status={:?} stdout={:?} stderr={:?}; \
                     consider preloading via /etc/modules-load.d/vkms.conf",
                    output.status.code(),
                    String::from_utf8_lossy(&output.stdout),
                    String::from_utf8_lossy(&output.stderr),
                ),
            });
        }

        self.kernel_module_loaded.store(true, Ordering::Relaxed);
        Ok(())
    }

    /// Allocate a fresh [`DisplayHandle`] and (if not offline) shell
    /// out to `xrandr --addmode` with the spec's resolution / refresh
    /// on the requested connector. Returns the handle so the caller
    /// can pass `handle.drm_card` to the Sunshine config emitter.
    pub async fn create_display(&mut self, spec: &DisplaySpec) -> Result<DisplayHandle> {
        self.ensure_module_loaded().await?;

        let id = DisplayId::new();
        let drm_card = if self.test_offline {
            PathBuf::from(OFFLINE_CARD)
        } else {
            discover_drm_card(&spec.connector)?
        };

        if !self.test_offline {
            let mode = format_mode(spec);
            debug!(connector = %spec.connector, mode = %mode, "xrandr --addmode");
            let output = Command::new("xrandr")
                .args(["--addmode", &spec.connector, &mode])
                .output()
                .await
                .map_err(|e| PhysicalError::DisplayUnavailable {
                    display: spec.connector.clone(),
                    reason: format!("failed to spawn xrandr: {e}"),
                })?;
            if !output.status.success() {
                return Err(PhysicalError::DisplayUnavailable {
                    display: spec.connector.clone(),
                    reason: format!(
                        "xrandr --addmode failed: status={:?} stdout={:?} stderr={:?}",
                        output.status.code(),
                        String::from_utf8_lossy(&output.stdout),
                        String::from_utf8_lossy(&output.stderr),
                    ),
                });
            }
        }

        let handle = DisplayHandle {
            id: id.clone(),
            spec: spec.clone(),
            drm_card,
        };
        info!(id = %handle.id, connector = %spec.connector, "virtual display created");
        self.active.insert(id, handle.clone());
        Ok(handle)
    }

    /// Drop a previously-created display. Idempotent — an unknown id
    /// is not an error. Shells `xrandr --delmode` for the connector.
    pub async fn destroy_display(&mut self, id: &DisplayId) -> Result<()> {
        let Some(handle) = self.active.remove(id) else {
            debug!(%id, "destroy_display: unknown id, no-op");
            return Ok(());
        };

        if self.test_offline {
            debug!(%id, "destroy_display: offline, skipping xrandr");
            return Ok(());
        }

        let mode = format_mode(&handle.spec);
        debug!(connector = %handle.spec.connector, mode = %mode, "xrandr --delmode");
        let output = Command::new("xrandr")
            .args(["--delmode", &handle.spec.connector, &mode])
            .output()
            .await
            .map_err(|e| PhysicalError::DisplayUnavailable {
                display: handle.spec.connector.clone(),
                reason: format!("failed to spawn xrandr --delmode: {e}"),
            })?;
        if !output.status.success() {
            return Err(PhysicalError::DisplayUnavailable {
                display: handle.spec.connector.clone(),
                reason: format!(
                    "xrandr --delmode failed: status={:?} stdout={:?} stderr={:?}",
                    output.status.code(),
                    String::from_utf8_lossy(&output.stdout),
                    String::from_utf8_lossy(&output.stderr),
                ),
            });
        }
        info!(%id, connector = %handle.spec.connector, "virtual display destroyed");
        Ok(())
    }

    /// Tear down every active display. Called from the parent actor's
    /// `post_stop`. Best-effort: logs each failure via [`warn!`] and
    /// returns Ok unless every teardown failed.
    pub async fn teardown_all(&mut self) -> Result<()> {
        let ids: Vec<DisplayId> = self.active.keys().cloned().collect();
        let total = ids.len();
        if total == 0 {
            return Ok(());
        }
        let mut failures = 0usize;
        for id in ids {
            if let Err(e) = self.destroy_display(&id).await {
                warn!(%id, error = %e, "teardown_all: destroy_display failed");
                failures += 1;
            }
        }
        if failures > 0 && failures == total {
            return Err(PhysicalError::DisplayUnavailable {
                display: "<all>".to_string(),
                reason: format!("every teardown failed ({failures}/{total})"),
            });
        }
        Ok(())
    }
}

/// Walk `/sys/class/drm` looking for a vkms-backed connector that
/// matches `connector`, returning the `/dev/dri/cardN` device path.
fn discover_drm_card(connector: &str) -> Result<PathBuf> {
    let root = Path::new(DRM_SYS_ROOT);
    let entries = std::fs::read_dir(root).map_err(|e| PhysicalError::DisplayUnavailable {
        display: connector.to_string(),
        reason: format!("read_dir {DRM_SYS_ROOT} failed: {e}"),
    })?;

    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        // Connector directories look like `card0-Virtual-1`.
        let Some(rest) = name.split_once('-').map(|(card, suffix)| (card.to_string(), suffix.to_string())) else {
            continue;
        };
        let (card, suffix) = rest;
        if suffix.eq_ignore_ascii_case(connector) {
            return Ok(PathBuf::from(format!("/dev/dri/{card}")));
        }
    }

    Err(PhysicalError::DisplayUnavailable {
        display: connector.to_string(),
        reason: format!(
            "no vkms-backed connector named {connector:?} found under {DRM_SYS_ROOT}; \
             is the module loaded and exposed?"
        ),
    })
}

/// Render a [`DisplaySpec`] as an `xrandr` mode string of the form
/// `WIDTHxHEIGHT_REFRESH`.
fn format_mode(spec: &DisplaySpec) -> String {
    let (w, h) = spec.resolution;
    format!("{w}x{h}_{}", spec.refresh_hz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_spec_hd_30_defaults() {
        let spec = DisplaySpec::hd_30();
        assert_eq!(spec.resolution, (1920, 1080));
        assert_eq!(spec.refresh_hz, 30);
        assert_eq!(spec.connector, "Virtual-1");
    }

    #[tokio::test]
    async fn ensure_module_loaded_offline_is_noop() {
        let mgr = VkmsDisplayManager::offline();
        mgr.ensure_module_loaded().await.unwrap();
        // Second call hits the memoised fast path.
        mgr.ensure_module_loaded().await.unwrap();
    }

    #[tokio::test]
    async fn offline_manager_create_destroy() {
        let mut mgr = VkmsDisplayManager::offline();
        let h1 = mgr
            .create_display(&DisplaySpec::new(1920, 1080, 30, "Virtual-1"))
            .await
            .unwrap();
        let _h2 = mgr
            .create_display(&DisplaySpec::new(1280, 720, 60, "Virtual-2"))
            .await
            .unwrap();
        assert_eq!(mgr.active_count(), 2);
        assert_eq!(h1.drm_card, PathBuf::from(OFFLINE_CARD));

        mgr.destroy_display(&h1.id).await.unwrap();
        assert_eq!(mgr.active_count(), 1);

        // Unknown id is a no-op, not an error.
        mgr.destroy_display(&DisplayId::new()).await.unwrap();
        assert_eq!(mgr.active_count(), 1);
    }

    #[tokio::test]
    async fn offline_manager_teardown_all_clears() {
        let mut mgr = VkmsDisplayManager::offline();
        for i in 0..3 {
            mgr.create_display(&DisplaySpec::new(1920, 1080, 30, format!("Virtual-{i}")))
                .await
                .unwrap();
        }
        assert_eq!(mgr.active_count(), 3);
        mgr.teardown_all().await.unwrap();
        assert_eq!(mgr.active_count(), 0);
    }
}
