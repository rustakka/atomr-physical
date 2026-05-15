//! mDNS service registration for spawned Sunshine instances.
//!
//! Remote Moonlight clients discover GameStream-compatible servers by
//! browsing the `_nvstream._tcp.local.` service type. The
//! [`MdnsBroadcaster`] in this module registers one such service per
//! live Sunshine instance through the
//! [`mdns-sd`](https://docs.rs/mdns-sd) crate, embedding the instance
//! id, stream bitrate, resolution, and frame rate in the service's
//! TXT record so a controller can pick the right stream without
//! probing the HTTP port first.
//!
//! Construct with [`MdnsBroadcaster::new`] in production or
//! [`MdnsBroadcaster::offline`] in unit + integration tests; the
//! offline form bookkeeps registrations purely in memory and never
//! touches the network.

use atomr_physical_core::{PhysicalError, Result, SunshineInstanceId};
use mdns_sd::{ServiceDaemon, ServiceInfo};
use std::collections::HashMap;
use std::sync::Mutex;
use tracing::{debug, info, warn};

/// Service-type string browsed by every GameStream-compatible
/// Moonlight client.
pub const NVSTREAM_SERVICE_TYPE: &str = "_nvstream._tcp.local.";

/// Length of the short instance suffix that distinguishes one
/// registered service from another on the same host label.
const SHORT_ID_LEN: usize = 8;

/// Prefix stripped from a [`SunshineInstanceId`] before building the
/// short suffix; matches the `sun-` newtype prefix minted by
/// [`SunshineInstanceId::new`](atomr_physical_core::SunshineInstanceId::new).
const INSTANCE_PREFIX: &str = "sun-";

/// One live mDNS registration for a Sunshine instance.
///
/// Returned by [`MdnsBroadcaster::register`] and stored inside the
/// broadcaster's registration map so [`unregister`](
/// MdnsBroadcaster::unregister) can find the full service name to
/// withdraw.
#[derive(Debug, Clone)]
pub struct MdnsRegistration {
    /// The Sunshine instance this registration advertises.
    pub instance: SunshineInstanceId,
    /// Full service name, e.g. `"atomr-abcdef01._nvstream._tcp.local."`.
    pub service_name: String,
    /// The Sunshine HTTP port this service is advertising as its
    /// primary port.
    pub http_port: u16,
    /// TXT record keys and values published with the service.
    pub txt: HashMap<String, String>,
}

/// Broadcasts each Sunshine instance as a `_nvstream._tcp.local.`
/// service so remote Moonlight nodes can discover it without hardcoded
/// IPs.
///
/// All mutating methods take `&self` so the parent
/// [`ProjectionActor`](crate::ProjectionActor) can call them from
/// inside its `handle()`; the registration map is protected by a
/// [`Mutex`]. [`shutdown`](Self::shutdown) takes `&mut self` so the
/// actor can drop the daemon cleanly in `post_stop`.
pub struct MdnsBroadcaster {
    /// `Some(daemon)` in production, `None` in offline / test mode.
    daemon: Option<ServiceDaemon>,
    host_label: String,
    registrations: Mutex<HashMap<SunshineInstanceId, MdnsRegistration>>,
}

impl MdnsBroadcaster {
    /// Production constructor — starts an [`mdns_sd::ServiceDaemon`].
    ///
    /// `host_label` is the prefix used to build per-instance service
    /// names: e.g. `"atomr"` yields
    /// `"atomr-abcdef01._nvstream._tcp.local."`.
    pub fn new(host_label: impl Into<String>) -> Result<Self> {
        let daemon = ServiceDaemon::new()
            .map_err(|e| PhysicalError::RemoteNode { reason: format!("mdns: {e}") })?;
        let host_label = host_label.into();
        info!(host_label, service_type = NVSTREAM_SERVICE_TYPE, "mDNS broadcaster started");
        Ok(Self {
            daemon: Some(daemon),
            host_label,
            registrations: Mutex::new(HashMap::new()),
        })
    }

    /// Test / offline constructor — every method bookkeeps
    /// registrations in memory without touching mDNS.
    pub fn offline(host_label: impl Into<String>) -> Self {
        let host_label = host_label.into();
        debug!(host_label, "mDNS broadcaster (offline) created");
        Self {
            daemon: None,
            host_label,
            registrations: Mutex::new(HashMap::new()),
        }
    }

    /// The host-label prefix used when minting service-instance names.
    pub fn host_label(&self) -> &str {
        &self.host_label
    }

    /// How many active registrations the broadcaster is holding.
    pub fn active_count(&self) -> usize {
        self.registrations
            .lock()
            .map(|g| g.len())
            .unwrap_or(0)
    }

    /// Register a Sunshine instance.
    ///
    /// Builds a service-instance name of the form
    /// `"<host_label>-<short_id>._nvstream._tcp.local."` where
    /// `short_id` is the last [`SHORT_ID_LEN`] characters of the
    /// instance id with the `sun-` prefix stripped (or the whole id if
    /// it is shorter than [`SHORT_ID_LEN`] characters).
    ///
    /// The TXT record carries `instance=<full_id>`,
    /// `bitrate=<kbps>`, `resolution=<w>x<h>`, and `fps=<n>` — the
    /// minimum a controller needs to pick the right stream off the
    /// LAN.
    pub fn register(
        &self,
        instance: &SunshineInstanceId,
        http_port: u16,
        bitrate_kbps: u32,
        resolution: (u32, u32),
        fps: u32,
    ) -> Result<MdnsRegistration> {
        let short = short_id_for(instance);
        let instance_name = format!("{}-{}", self.host_label, short);
        let service_name = format!("{instance_name}.{NVSTREAM_SERVICE_TYPE}");
        let host_name = format!("{instance_name}.local.");

        let full_id = instance.as_str().to_string();
        let bitrate_str = bitrate_kbps.to_string();
        let resolution_str = format!("{}x{}", resolution.0, resolution.1);
        let fps_str = fps.to_string();

        let txt_pairs: &[(&str, &str); 4] = &[
            ("instance", full_id.as_str()),
            ("bitrate", bitrate_str.as_str()),
            ("resolution", resolution_str.as_str()),
            ("fps", fps_str.as_str()),
        ];

        let mut txt = HashMap::new();
        for (k, v) in txt_pairs.iter() {
            txt.insert((*k).to_string(), (*v).to_string());
        }

        if let Some(daemon) = self.daemon.as_ref() {
            // `ip_or_empty` left blank — mdns-sd will publish every
            // non-loopback interface address by default, which is what
            // we want for LAN discovery.
            let info = ServiceInfo::new(
                NVSTREAM_SERVICE_TYPE,
                instance_name.as_str(),
                host_name.as_str(),
                "",
                http_port,
                &txt_pairs[..],
            )
            .map_err(|e| PhysicalError::RemoteNode { reason: format!("mdns: {e}") })?;

            daemon
                .register(info)
                .map_err(|e| PhysicalError::RemoteNode { reason: format!("mdns: {e}") })?;
            info!(
                instance = %instance,
                service_name = %service_name,
                http_port,
                "mDNS service registered"
            );
        } else {
            debug!(
                instance = %instance,
                service_name = %service_name,
                http_port,
                "mDNS service registered (offline)"
            );
        }

        let reg = MdnsRegistration {
            instance: instance.clone(),
            service_name,
            http_port,
            txt,
        };

        let mut guard = self
            .registrations
            .lock()
            .map_err(|e| PhysicalError::RemoteNode { reason: format!("mdns: lock poisoned: {e}") })?;
        guard.insert(instance.clone(), reg.clone());
        Ok(reg)
    }

    /// Unregister a previously-registered instance. Idempotent: if no
    /// registration exists for `instance`, returns `Ok(())`.
    pub fn unregister(&self, instance: &SunshineInstanceId) -> Result<()> {
        let removed = {
            let mut guard = self.registrations.lock().map_err(|e| PhysicalError::RemoteNode {
                reason: format!("mdns: lock poisoned: {e}"),
            })?;
            guard.remove(instance)
        };

        let Some(reg) = removed else {
            debug!(instance = %instance, "unregister: nothing to do");
            return Ok(());
        };

        if let Some(daemon) = self.daemon.as_ref() {
            // mdns-sd 0.11's sync `unregister` returns a
            // Receiver<UnregisterStatus>; we don't wait on it — the
            // daemon will flush the goodbye packets in the background
            // and dropping the receiver is supported.
            match daemon.unregister(&reg.service_name) {
                Ok(_rx) => {
                    info!(instance = %instance, service_name = %reg.service_name, "mDNS service unregistered");
                }
                Err(e) => {
                    warn!(
                        instance = %instance,
                        service_name = %reg.service_name,
                        error = %e,
                        "mDNS unregister returned error; in-memory record already dropped"
                    );
                }
            }
        } else {
            debug!(instance = %instance, service_name = %reg.service_name, "mDNS service unregistered (offline)");
        }
        Ok(())
    }

    /// Tear down the daemon and drop every registration. Called from
    /// the parent actor's `post_stop`.
    pub fn shutdown(&mut self) -> Result<()> {
        let instances: Vec<SunshineInstanceId> = {
            let guard = self.registrations.lock().map_err(|e| PhysicalError::RemoteNode {
                reason: format!("mdns: lock poisoned: {e}"),
            })?;
            guard.keys().cloned().collect()
        };
        for id in instances {
            if let Err(e) = self.unregister(&id) {
                warn!(instance = %id, error = %e, "shutdown: unregister failed");
            }
        }

        if let Some(daemon) = self.daemon.take() {
            match daemon.shutdown() {
                Ok(_rx) => {
                    info!(host_label = %self.host_label, "mDNS broadcaster shut down");
                }
                Err(e) => {
                    warn!(host_label = %self.host_label, error = %e, "mDNS daemon shutdown returned error");
                }
            }
        } else {
            debug!(host_label = %self.host_label, "mDNS broadcaster (offline) shut down");
        }
        Ok(())
    }
}

/// Build the short suffix used in a service-instance name from a
/// [`SunshineInstanceId`]: strip the `sun-` prefix, then take the last
/// [`SHORT_ID_LEN`] characters of the remainder (or the whole id if it
/// has fewer characters than that).
fn short_id_for(instance: &SunshineInstanceId) -> String {
    let raw = instance.as_str();
    let stripped = raw.strip_prefix(INSTANCE_PREFIX).unwrap_or(raw);
    let chars: Vec<char> = stripped.chars().collect();
    if chars.len() <= SHORT_ID_LEN {
        return stripped.to_string();
    }
    chars[chars.len() - SHORT_ID_LEN..].iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iid(s: &str) -> SunshineInstanceId {
        SunshineInstanceId::from(s)
    }

    #[test]
    fn short_id_strips_prefix_and_takes_tail() {
        let id = iid("sun-abcdef0123456789");
        assert_eq!(short_id_for(&id), "23456789");
    }

    #[test]
    fn short_id_handles_no_prefix() {
        // No `sun-` prefix: nothing is stripped, the last SHORT_ID_LEN
        // characters of the raw id are returned.
        let id = iid("abcdef0123");
        assert_eq!(short_id_for(&id), "cdef0123");
    }

    #[test]
    fn short_id_short_input_returned_whole() {
        let id = iid("sun-abc");
        assert_eq!(short_id_for(&id), "abc");
    }

    #[test]
    fn offline_broadcaster_register_unregister() {
        let bcast = MdnsBroadcaster::offline("atomr");
        let id1 = iid("sun-aaaaaaaa11111111");
        let id2 = iid("sun-bbbbbbbb22222222");

        bcast.register(&id1, 47989, 20_000, (1920, 1080), 60).unwrap();
        bcast.register(&id2, 47990, 20_000, (1280, 720), 60).unwrap();
        assert_eq!(bcast.active_count(), 2);

        bcast.unregister(&id1).unwrap();
        assert_eq!(bcast.active_count(), 1);
    }

    #[test]
    fn offline_broadcaster_register_returns_full_service_name() {
        let bcast = MdnsBroadcaster::offline("atomr");
        let id = iid("sun-aaaaaaaa11111111");
        let reg = bcast.register(&id, 47989, 20_000, (1920, 1080), 60).unwrap();

        assert!(reg.service_name.ends_with(NVSTREAM_SERVICE_TYPE));
        assert!(reg.service_name.starts_with("atomr-"));
        assert_eq!(reg.instance, id);
        assert_eq!(reg.http_port, 47989);
    }

    #[test]
    fn offline_broadcaster_txt_contains_all_keys() {
        let bcast = MdnsBroadcaster::offline("atomr");
        let id = iid("sun-aaaaaaaa11111111");
        let reg = bcast.register(&id, 47989, 25_000, (1920, 1080), 30).unwrap();

        assert_eq!(reg.txt.get("instance").map(|s| s.as_str()), Some("sun-aaaaaaaa11111111"));
        assert_eq!(reg.txt.get("bitrate").map(|s| s.as_str()), Some("25000"));
        assert_eq!(reg.txt.get("resolution").map(|s| s.as_str()), Some("1920x1080"));
        assert_eq!(reg.txt.get("fps").map(|s| s.as_str()), Some("30"));
    }

    #[test]
    fn unregister_unknown_is_ok() {
        let bcast = MdnsBroadcaster::offline("atomr");
        bcast.unregister(&iid("sun-never-registered")).unwrap();
        assert_eq!(bcast.active_count(), 0);
    }

    #[test]
    fn host_label_accessor_round_trips() {
        let bcast = MdnsBroadcaster::offline("rover-1");
        assert_eq!(bcast.host_label(), "rover-1");
    }

    #[test]
    fn shutdown_drops_all_registrations() {
        let mut bcast = MdnsBroadcaster::offline("atomr");
        for i in 0..3 {
            bcast
                .register(
                    &iid(&format!("sun-aaaaaaaa1111111{i}")),
                    47989 + i as u16,
                    20_000,
                    (1920, 1080),
                    60,
                )
                .unwrap();
        }
        assert_eq!(bcast.active_count(), 3);
        bcast.shutdown().unwrap();
        assert_eq!(bcast.active_count(), 0);
    }
}
