//! [`ProjectionActor`] — the supervisor at the top of a projection
//! subtree.
//!
//! Construct one with [`ProjectionActor::new`], builder-tune it
//! (encoder, bitrate tiers, offline/test mode, supervisor strategy),
//! then call [`spawn`](ProjectionActor::spawn) or
//! [`spawn_under`](ProjectionActor::spawn_under) to promote it to a
//! live atomr actor. The returned [`ProjectionActorRef`] is the typed
//! handle that downstream code uses to request projections, pair
//! clients, and tear instances down.
//!
//! The actor owns four cooperating subsystems:
//!
//! 1. A [`VkmsDisplayManager`](crate::VkmsDisplayManager) that brings
//!    up and tears down vkms-backed virtual displays.
//! 2. A [`PortAllocator`](crate::PortAllocator) that hands out
//!    stride-shifted port windows to stacked Sunshine instances.
//! 3. A pool of supervised [`SunshineInstanceActor`](crate::SunshineInstanceActor)
//!    children — one `sunshine` process per active projection.
//! 4. A [`ClientProvisioner`](crate::ClientProvisioner) + an
//!    [`MdnsBroadcaster`](crate::MdnsBroadcaster) that together do
//!    discovery + automated Moonlight pairing.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_core::supervision::{OneForOneStrategy, SupervisorStrategy};
use atomr_physical_core::{
    ClientId, DisplayId, PhysicalError, ProjectionId, Result, SunshineInstanceId,
};
use serde::{Deserialize, Serialize};
use tokio::sync::oneshot;
use url::Url;

use crate::config_template::SunshineConfigParams;
use crate::display::{DisplaySpec, VkmsDisplayManager};
use crate::mdns::{MdnsBroadcaster, MdnsRegistration};
use crate::pairing::{ClientProvisioner, PairingRecord};
use crate::ports::{PortAllocator, PortWindow};
use crate::sunshine::{SunshineInstanceActor, SunshineInstanceRef, SunshineInstanceSummary};

/// A bitrate tier applied based on the number of mirrored clients.
///
/// When the live client count for a Sunshine instance crosses one of
/// these thresholds, the supervisor performs a graceful restart of
/// that instance with the new bitrate baked into its config.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct BandwidthTier {
    /// The (inclusive) lower bound of the client count this tier applies to.
    pub clients_at_least: u16,
    /// Bitrate for the tier, in kbps.
    pub bitrate_kbps: u32,
}

impl BandwidthTier {
    /// Construct a tier.
    pub fn new(clients_at_least: u16, bitrate_kbps: u32) -> Self {
        Self {
            clients_at_least,
            bitrate_kbps,
        }
    }
}

/// Per-request projection parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectionSpec {
    /// Virtual-display layout (resolution + connector name).
    pub display: DisplaySpec,
    /// Sunshine encoder name: `software`, `nvenc`, `vaapi`, `quicksync`.
    pub encoder: String,
    /// Initial bitrate in kbps (overridden by [`BandwidthTier`] on mirror).
    pub bitrate_kbps: u32,
    /// Sunshine's `min_log_bitrate` (kbps).
    pub min_log_bitrate_kbps: u32,
    /// Sunshine's `max_bitrate` (kbps).
    pub max_bitrate_kbps: u32,
    /// Frames per second.
    pub fps: u32,
}

impl ProjectionSpec {
    /// A safe single-client 1080p30 software-encoded default.
    pub fn defaults() -> Self {
        Self {
            display: DisplaySpec::hd_30(),
            encoder: "software".to_string(),
            bitrate_kbps: 20_000,
            min_log_bitrate_kbps: 1_000,
            max_bitrate_kbps: 20_000,
            fps: 30,
        }
    }
}

/// Handle returned by [`ProjectionActorRef::request_projection`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectionHandle {
    /// Stable identifier the caller uses for follow-up calls.
    pub projection_id: ProjectionId,
    /// The Sunshine instance backing this projection.
    pub instance_id: SunshineInstanceId,
    /// The virtual display the instance is streaming.
    pub display_id: DisplayId,
    /// The reserved Sunshine port window.
    pub port_window: PortWindow,
    /// Fully qualified mDNS service name (e.g. `atomr-abc12345._nvstream._tcp.local.`).
    pub mdns_service: String,
}

/// Result of a successful pairing handshake initiation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PairingTicket {
    /// The instance this ticket pairs against.
    pub instance: SunshineInstanceId,
    /// The client identifier the ticket was issued to.
    pub client_id: ClientId,
    /// Server-issued salt bytes (opaque — pass back through [`ProjectionActorRef::submit_pin`]).
    pub salt: Vec<u8>,
}

/// The mailbox protocol of a live [`ProjectionActor`].
///
/// Callers should reach for [`ProjectionActorRef`] rather than
/// constructing these variants directly — the ref wraps the oneshot
/// replies and the ask timeout.
pub enum ProjectionMsg {
    /// Bring a fresh projection online: allocate a port window, create
    /// a virtual display, spawn a supervised Sunshine instance, and
    /// broadcast the instance over mDNS.
    RequestProjection {
        /// Per-request parameters.
        spec: ProjectionSpec,
        /// One-shot reply.
        reply: oneshot::Sender<Result<ProjectionHandle>>,
    },
    /// Start the pairing handshake for a remote client.
    PairClient {
        /// Sunshine instance to pair against.
        instance: SunshineInstanceId,
        /// Caller-chosen client identifier.
        client_id: ClientId,
        /// Display name for the client (free-form).
        hostname: String,
        /// One-shot reply.
        reply: oneshot::Sender<Result<PairingTicket>>,
    },
    /// Submit the PIN that completes a pairing started by `PairClient`.
    SubmitPin {
        /// Sunshine instance to pair against.
        instance: SunshineInstanceId,
        /// The client this PIN is for.
        client_id: ClientId,
        /// Hostname previously passed to `PairClient`.
        hostname: String,
        /// The PIN string (4 digits, ASCII).
        pin: String,
        /// One-shot reply.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Snapshot every supervised instance's summary.
    ListInstances {
        /// One-shot reply.
        reply: oneshot::Sender<Result<Vec<SunshineInstanceSummary>>>,
    },
    /// Look up the [`ProjectionHandle`] for a previously requested
    /// projection.
    LookupHandle {
        /// The projection to resolve.
        projection_id: ProjectionId,
        /// One-shot reply (None if not registered).
        reply: oneshot::Sender<Option<ProjectionHandle>>,
    },
    /// Gracefully tear down a Sunshine instance — SIGTERM the process,
    /// drop the virtual display, free the port window, unregister
    /// from mDNS, and forget any pairings.
    StopInstance {
        /// Sunshine instance to stop.
        instance: SunshineInstanceId,
        /// One-shot reply.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Snapshot the in-memory pairing book.
    KnownPairings {
        /// One-shot reply.
        reply: oneshot::Sender<Vec<PairingRecord>>,
    },
}

/// Builder / spec struct for a [`ProjectionActor`].
///
/// `Clone` is cheap.
#[derive(Clone)]
pub struct ProjectionActor {
    id: ProjectionId,
    sunshine_binary: PathBuf,
    mdns_host_label: String,
    accept_self_signed: bool,
    test_offline: bool,
    bandwidth_thresholds: Vec<BandwidthTier>,
    supervisor_strategy: SupervisorStrategy,
}

impl ProjectionActor {
    /// Construct a fresh projection supervisor with the given Sunshine
    /// server binary.
    ///
    /// Production deployments pass `/usr/bin/sunshine`. Tests and the
    /// `project demo` CLI subcommand pass `/bin/sleep` together with
    /// [`with_test_offline(true)`](Self::with_test_offline) to bypass
    /// every shell-out (display, mDNS, pairing).
    pub fn new(sunshine_binary: PathBuf) -> Self {
        Self {
            id: ProjectionId::new(),
            sunshine_binary,
            mdns_host_label: "atomr".to_string(),
            accept_self_signed: true,
            test_offline: false,
            bandwidth_thresholds: vec![
                BandwidthTier::new(1, 20_000),
                BandwidthTier::new(2, 10_000),
                BandwidthTier::new(3, 6_000),
            ],
            supervisor_strategy: OneForOneStrategy::default().into(),
        }
    }

    /// Builder-style: override the projection id.
    pub fn with_id(mut self, id: ProjectionId) -> Self {
        self.id = id;
        self
    }

    /// Builder-style: change the mDNS host label (the prefix on every
    /// service-instance name).
    pub fn with_mdns_host_label(mut self, label: impl Into<String>) -> Self {
        self.mdns_host_label = label.into();
        self
    }

    /// Builder-style: enable / disable TOFU TLS acceptance for the
    /// pairing client (default: true).
    pub fn with_accept_self_signed(mut self, accept: bool) -> Self {
        self.accept_self_signed = accept;
        self
    }

    /// Builder-style: enable the test/offline pathway — every shell-out
    /// is short-circuited and the actor uses the `/bin/sleep`-friendly
    /// instance launcher. Required for the CLI demo and integration
    /// tests.
    pub fn with_test_offline(mut self, offline: bool) -> Self {
        self.test_offline = offline;
        self
    }

    /// Builder-style: replace the bandwidth tier table used for
    /// graceful-restart-on-mirror.
    pub fn with_bandwidth_thresholds(mut self, tiers: Vec<BandwidthTier>) -> Self {
        self.bandwidth_thresholds = tiers;
        self
    }

    /// Builder-style: customise the atomr supervisor strategy applied
    /// to spawned Sunshine instances. Defaults to one-for-one restart
    /// with 10 retries / 60 s.
    pub fn with_supervisor_strategy(mut self, strategy: SupervisorStrategy) -> Self {
        self.supervisor_strategy = strategy;
        self
    }

    /// The projection id this supervisor was built with.
    pub fn id(&self) -> &ProjectionId {
        &self.id
    }

    /// The Sunshine binary path the supervisor will spawn.
    pub fn sunshine_binary(&self) -> &PathBuf {
        &self.sunshine_binary
    }

    /// Whether the supervisor is configured for offline / test mode.
    pub fn test_offline(&self) -> bool {
        self.test_offline
    }

    /// Promote this supervisor into a top-level atomr actor.
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<ProjectionActorRef, ActorSystemError> {
        let (props, id) = self.into_runner_props();
        let inner = system.actor_of(props, name)?;
        Ok(ProjectionActorRef { inner, id })
    }

    /// Promote this supervisor into a supervised child of `P`.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<ProjectionActorRef> {
        let (props, id) = self.into_runner_props();
        let inner = ctx
            .spawn(props, name)
            .map_err(|e| PhysicalError::Fault(format!("projection child spawn failed: {e}")))?;
        Ok(ProjectionActorRef { inner, id })
    }

    fn into_runner_props(self) -> (Props<ProjectionRunner>, ProjectionId) {
        let id = self.id.clone();
        let supervisor_strategy = self.supervisor_strategy.clone();
        let sunshine_binary = self.sunshine_binary;
        let mdns_host_label = self.mdns_host_label;
        let accept_self_signed = self.accept_self_signed;
        let test_offline = self.test_offline;
        let bandwidth_thresholds = self.bandwidth_thresholds;
        let id_factory = id.clone();
        let strategy_factory = supervisor_strategy.clone();
        let port_map: PortMap = Arc::new(RwLock::new(HashMap::new()));
        let props = Props::create(move || {
            ProjectionRunner::new(RunnerSeed {
                id: id_factory.clone(),
                sunshine_binary: sunshine_binary.clone(),
                mdns_host_label: mdns_host_label.clone(),
                accept_self_signed,
                test_offline,
                bandwidth_thresholds: bandwidth_thresholds.clone(),
                port_map: port_map.clone(),
            })
        })
        .with_supervisor_strategy(strategy_factory);
        (props, id)
    }
}

/// A typed handle to a spawned [`ProjectionActor`].
#[derive(Clone)]
pub struct ProjectionActorRef {
    inner: ActorRef<ProjectionMsg>,
    id: ProjectionId,
}

impl ProjectionActorRef {
    /// The projection id.
    pub fn id(&self) -> &ProjectionId {
        &self.id
    }

    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<ProjectionMsg> {
        &self.inner
    }

    /// Request a fresh projection.
    pub async fn request_projection(&self, spec: ProjectionSpec) -> Result<ProjectionHandle> {
        self.inner
            .ask_with(
                |reply| ProjectionMsg::RequestProjection { spec, reply },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)?
    }

    /// Begin a pairing handshake for `client_id` against `instance`.
    pub async fn pair_client(
        &self,
        instance: SunshineInstanceId,
        client_id: ClientId,
        hostname: String,
    ) -> Result<PairingTicket> {
        self.inner
            .ask_with(
                |reply| ProjectionMsg::PairClient {
                    instance,
                    client_id,
                    hostname,
                    reply,
                },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)?
    }

    /// Complete a pairing handshake.
    pub async fn submit_pin(
        &self,
        instance: SunshineInstanceId,
        client_id: ClientId,
        hostname: String,
        pin: String,
    ) -> Result<()> {
        self.inner
            .ask_with(
                |reply| ProjectionMsg::SubmitPin {
                    instance,
                    client_id,
                    hostname,
                    pin,
                    reply,
                },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)?
    }

    /// Snapshot every supervised Sunshine instance's summary.
    pub async fn list_instances(&self) -> Result<Vec<SunshineInstanceSummary>> {
        self.inner
            .ask_with(|reply| ProjectionMsg::ListInstances { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }

    /// Look up the [`ProjectionHandle`] for a projection id.
    pub async fn lookup_handle(&self, projection_id: ProjectionId) -> Result<Option<ProjectionHandle>> {
        self.inner
            .ask_with(
                |reply| ProjectionMsg::LookupHandle {
                    projection_id,
                    reply,
                },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)
    }

    /// Gracefully stop a Sunshine instance and clean up its display,
    /// port window, and mDNS registration.
    pub async fn stop_instance(&self, instance: SunshineInstanceId) -> Result<()> {
        self.inner
            .ask_with(
                |reply| ProjectionMsg::StopInstance { instance, reply },
                ASK_TIMEOUT,
            )
            .await
            .map_err(ask_to_physical)?
    }

    /// Snapshot the in-memory pairing book.
    pub async fn known_pairings(&self) -> Result<Vec<PairingRecord>> {
        self.inner
            .ask_with(|reply| ProjectionMsg::KnownPairings { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }
}

const ASK_TIMEOUT: Duration = Duration::from_secs(5);

fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("projection actor ask failed: {e:?}"))
}

/// Shared port-window map used by the pairing client's base-URL
/// resolver. The map is mutated from inside the actor on instance
/// create/destroy and read from the provisioner closure when issuing
/// HTTPS calls to Sunshine.
type PortMap = Arc<RwLock<HashMap<SunshineInstanceId, PortWindow>>>;

struct RunnerSeed {
    id: ProjectionId,
    sunshine_binary: PathBuf,
    mdns_host_label: String,
    accept_self_signed: bool,
    test_offline: bool,
    bandwidth_thresholds: Vec<BandwidthTier>,
    port_map: PortMap,
}

/// One row in the runner's instance registry.
#[allow(dead_code)] // `mdns` and `spec` are read by future bandwidth-tier restart logic.
struct InstanceState {
    sunshine: SunshineInstanceRef,
    display: DisplayId,
    port_window: PortWindow,
    mdns: MdnsRegistration,
    projection: ProjectionId,
    spec: ProjectionSpec,
    clients: u16,
}

/// Internal supervisor implementation backing a spawned
/// [`ProjectionActor`].
struct ProjectionRunner {
    id: ProjectionId,
    sunshine_binary: PathBuf,
    mdns_host_label: String,
    accept_self_signed: bool,
    test_offline: bool,
    bandwidth_thresholds: Vec<BandwidthTier>,
    displays: VkmsDisplayManager,
    ports: PortAllocator,
    mdns: MdnsBroadcaster,
    provisioner: Arc<ClientProvisioner>,
    instances: HashMap<SunshineInstanceId, InstanceState>,
    handles: HashMap<ProjectionId, ProjectionHandle>,
    port_map: PortMap,
}

impl ProjectionRunner {
    fn new(seed: RunnerSeed) -> Self {
        let RunnerSeed {
            id,
            sunshine_binary,
            mdns_host_label,
            accept_self_signed,
            test_offline,
            bandwidth_thresholds,
            port_map,
        } = seed;
        let displays = if test_offline {
            VkmsDisplayManager::offline()
        } else {
            VkmsDisplayManager::new(false)
        };
        let ports = PortAllocator::new();
        let mdns = if test_offline {
            MdnsBroadcaster::offline(mdns_host_label.clone())
        } else {
            // Failures here become a Fault on first message; we can't
            // surface them from a factory closure, so log + fall back
            // to the offline form.
            MdnsBroadcaster::new(mdns_host_label.clone()).unwrap_or_else(|e| {
                tracing::warn!(
                    projection = %id,
                    error = %e,
                    "mDNS daemon failed to start — falling back to offline broadcaster"
                );
                MdnsBroadcaster::offline(mdns_host_label.clone())
            })
        };
        let provisioner = if test_offline {
            Arc::new(ClientProvisioner::offline())
        } else {
            let lookup = port_map.clone();
            let built = ClientProvisioner::new(
                move |instance: &SunshineInstanceId| {
                    let port = lookup
                        .read()
                        .ok()
                        .and_then(|guard| guard.get(instance).map(|w| w.http_port()))
                        .unwrap_or(47989);
                    Url::parse(&format!("https://127.0.0.1:{port}/"))
                        .expect("static URL parses")
                },
                accept_self_signed,
            )
            .unwrap_or_else(|e| {
                tracing::warn!(
                    projection = %id,
                    error = %e,
                    "pairing client failed to build — falling back to offline provisioner"
                );
                ClientProvisioner::offline()
            });
            Arc::new(built)
        };
        Self {
            id,
            sunshine_binary,
            mdns_host_label,
            accept_self_signed,
            test_offline,
            bandwidth_thresholds,
            displays,
            ports,
            mdns,
            provisioner,
            instances: HashMap::new(),
            handles: HashMap::new(),
            port_map,
        }
    }

    fn pick_bitrate(&self, clients: u16, requested: u32) -> u32 {
        let tier = self
            .bandwidth_thresholds
            .iter()
            .filter(|t| t.clients_at_least <= clients.max(1))
            .max_by_key(|t| t.clients_at_least);
        tier.map(|t| t.bitrate_kbps).unwrap_or(requested)
    }

    async fn create_projection(
        &mut self,
        ctx: &mut Context<Self>,
        spec: ProjectionSpec,
    ) -> Result<ProjectionHandle> {
        // 1. Bring vkms up (no-op in offline mode).
        self.displays.ensure_module_loaded().await?;
        // 2. Reserve a port window for the new Sunshine instance.
        let port_window = self.ports.reserve()?;
        // 3. Bring a virtual display online.
        let display_handle = match self.displays.create_display(&spec.display).await {
            Ok(h) => h,
            Err(e) => {
                self.ports.release(&port_window);
                return Err(e);
            }
        };
        // 4. Synthesize the instance id + config.
        let instance_id = SunshineInstanceId::new();
        let projection_id = ProjectionId::new();
        let bitrate = self.pick_bitrate(1, spec.bitrate_kbps);
        let mut params = SunshineConfigParams::defaults_for(
            instance_id.clone(),
            port_window,
            display_handle.drm_card.clone(),
        );
        params.display_name = display_handle.spec.connector.clone();
        params.encoder = spec.encoder.clone();
        params.bitrate_kbps = bitrate;
        params.min_log_bitrate_kbps = spec.min_log_bitrate_kbps;
        params.max_bitrate_kbps = spec.max_bitrate_kbps;
        params.fps = spec.fps;
        params.resolution = spec.display.resolution;
        // 5. Spawn the supervised SunshineInstanceActor as a child.
        let mut instance_actor =
            SunshineInstanceActor::new(instance_id.clone(), self.sunshine_binary.clone(), params);
        if self.test_offline {
            instance_actor = instance_actor.with_skip_config_arg(true);
            // Make /bin/sleep cooperate: pass a duration if no extra
            // args are present.
            instance_actor = instance_actor.with_extra_args(vec!["3600".to_string()]);
        }
        let child_name = format!("sunshine-{}", instance_id.as_str());
        let sunshine_ref = match instance_actor.spawn_under(ctx, &child_name) {
            Ok(r) => r,
            Err(e) => {
                self.ports.release(&port_window);
                let _ = self.displays.destroy_display(&display_handle.id).await;
                return Err(e);
            }
        };
        // 6. Register the instance on mDNS.
        let mdns_reg = match self.mdns.register(
            &instance_id,
            port_window.http_port(),
            bitrate,
            spec.display.resolution,
            spec.fps,
        ) {
            Ok(r) => r,
            Err(e) => {
                // Best-effort: tear the instance back down.
                let _ = sunshine_ref.shutdown().await;
                self.ports.release(&port_window);
                let _ = self.displays.destroy_display(&display_handle.id).await;
                return Err(e);
            }
        };
        // 7. Make the port window visible to the pairing client.
        if let Ok(mut guard) = self.port_map.write() {
            guard.insert(instance_id.clone(), port_window);
        }
        // 8. Stash the live state + the public handle.
        let handle = ProjectionHandle {
            projection_id: projection_id.clone(),
            instance_id: instance_id.clone(),
            display_id: display_handle.id.clone(),
            port_window,
            mdns_service: mdns_reg.service_name.clone(),
        };
        self.instances.insert(
            instance_id.clone(),
            InstanceState {
                sunshine: sunshine_ref,
                display: display_handle.id,
                port_window,
                mdns: mdns_reg,
                projection: projection_id.clone(),
                spec,
                clients: 0,
            },
        );
        self.handles.insert(projection_id, handle.clone());
        tracing::info!(
            projection = %self.id,
            instance = %instance_id,
            mdns = %handle.mdns_service,
            tcp = ?port_window.tcp,
            udp = ?port_window.udp,
            "projection online"
        );
        Ok(handle)
    }

    async fn stop_instance(&mut self, instance_id: SunshineInstanceId) -> Result<()> {
        let Some(state) = self.instances.remove(&instance_id) else {
            return Ok(()); // Idempotent: unknown instance is a no-op.
        };
        let _ = state.sunshine.shutdown().await;
        let _ = self.mdns.unregister(&instance_id);
        let _ = self.displays.destroy_display(&state.display).await;
        self.ports.release(&state.port_window);
        if let Ok(mut guard) = self.port_map.write() {
            guard.remove(&instance_id);
        }
        self.handles.remove(&state.projection);
        tracing::info!(
            projection = %self.id,
            instance = %instance_id,
            "projection torn down"
        );
        Ok(())
    }
}

#[async_trait]
impl Actor for ProjectionRunner {
    type Msg = ProjectionMsg;

    fn supervisor_strategy(&self) -> SupervisorStrategy {
        OneForOneStrategy::default().into()
    }

    async fn pre_start(&mut self, _ctx: &mut Context<Self>) {
        // Touch the kernel module probe eagerly so the first projection
        // request doesn't pay that cost. Offline mode short-circuits.
        if let Err(e) = self.displays.ensure_module_loaded().await {
            tracing::warn!(
                projection = %self.id,
                error = %e,
                "vkms not currently loaded; first projection request may fail"
            );
        }
        tracing::info!(
            projection = %self.id,
            sunshine_binary = ?self.sunshine_binary,
            mdns_host = %self.mdns_host_label,
            offline = self.test_offline,
            accept_self_signed = self.accept_self_signed,
            "projection supervisor started"
        );
    }

    async fn handle(&mut self, ctx: &mut Context<Self>, msg: ProjectionMsg) {
        match msg {
            ProjectionMsg::RequestProjection { spec, reply } => {
                let outcome = self.create_projection(ctx, spec).await;
                let _ = reply.send(outcome);
            }
            ProjectionMsg::PairClient {
                instance,
                client_id,
                hostname,
                reply,
            } => {
                let provisioner = self.provisioner.clone();
                let result = provisioner
                    .start_pairing(&instance, &client_id, &hostname)
                    .await
                    .map(|salt| PairingTicket {
                        instance: instance.clone(),
                        client_id: client_id.clone(),
                        salt,
                    });
                if let Some(state) = self.instances.get_mut(&instance) {
                    if result.is_ok() {
                        state.clients = state.clients.saturating_add(1);
                    }
                }
                let _ = reply.send(result);
            }
            ProjectionMsg::SubmitPin {
                instance,
                client_id,
                hostname,
                pin,
                reply,
            } => {
                let provisioner = self.provisioner.clone();
                let result = provisioner
                    .submit_pin(&instance, &client_id, &hostname, &pin)
                    .await;
                let _ = reply.send(result);
            }
            ProjectionMsg::ListInstances { reply } => {
                let refs: Vec<SunshineInstanceRef> = self
                    .instances
                    .values()
                    .map(|s| s.sunshine.clone())
                    .collect();
                let mut out = Vec::with_capacity(refs.len());
                let mut first_err: Option<PhysicalError> = None;
                for r in refs {
                    match r.summary().await {
                        Ok(s) => out.push(s),
                        Err(e) => {
                            if first_err.is_none() {
                                first_err = Some(e);
                            }
                        }
                    }
                }
                let outcome = match first_err {
                    Some(e) if out.is_empty() => Err(e),
                    _ => Ok(out),
                };
                let _ = reply.send(outcome);
            }
            ProjectionMsg::LookupHandle {
                projection_id,
                reply,
            } => {
                let _ = reply.send(self.handles.get(&projection_id).cloned());
            }
            ProjectionMsg::StopInstance { instance, reply } => {
                let outcome = self.stop_instance(instance).await;
                let _ = reply.send(outcome);
            }
            ProjectionMsg::KnownPairings { reply } => {
                let provisioner = self.provisioner.clone();
                let snap = provisioner.known_pairings().await;
                let _ = reply.send(snap);
            }
        }
    }

    async fn post_stop(&mut self, _ctx: &mut Context<Self>) {
        // Tear instances down in arbitrary order — each is independent.
        let ids: Vec<SunshineInstanceId> = self.instances.keys().cloned().collect();
        for id in ids {
            let _ = self.stop_instance(id).await;
        }
        let _ = self.mdns.shutdown();
        let _ = self.displays.teardown_all().await;
        tracing::info!(projection = %self.id, "projection supervisor stopped");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_core::actor::ActorSystem;
    use std::time::Duration;

    fn config() -> atomr_config::Config {
        atomr_config::Config::reference()
    }

    #[tokio::test]
    async fn projection_actor_request_creates_instance() {
        let sys = ActorSystem::create("projection-request", config()).await.unwrap();
        let actor = ProjectionActor::new(PathBuf::from("/bin/sleep"))
            .with_test_offline(true)
            .with_mdns_host_label("atomr-test");
        let actor_ref = actor.spawn(&sys, "projection-1").unwrap();
        let handle = actor_ref
            .request_projection(ProjectionSpec::defaults())
            .await
            .unwrap();
        assert!(handle.mdns_service.starts_with("atomr-test-"));
        assert!(handle.mdns_service.ends_with("._nvstream._tcp.local."));
        // Give the child a moment to land in Running.
        tokio::time::sleep(Duration::from_millis(200)).await;
        let summaries = actor_ref.list_instances().await.unwrap();
        assert_eq!(summaries.len(), 1);
        assert_eq!(summaries[0].id, handle.instance_id);
        sys.terminate().await;
    }

    #[tokio::test]
    async fn projection_actor_lookup_handle_returns_registered() {
        let sys = ActorSystem::create("projection-lookup", config()).await.unwrap();
        let actor_ref = ProjectionActor::new(PathBuf::from("/bin/sleep"))
            .with_test_offline(true)
            .spawn(&sys, "projection-lookup")
            .unwrap();
        let handle = actor_ref
            .request_projection(ProjectionSpec::defaults())
            .await
            .unwrap();
        let found = actor_ref.lookup_handle(handle.projection_id.clone()).await.unwrap();
        assert!(found.is_some());
        let missing = actor_ref.lookup_handle(ProjectionId::new()).await.unwrap();
        assert!(missing.is_none());
        sys.terminate().await;
    }

    #[tokio::test]
    async fn projection_actor_stop_instance_cleans_state() {
        let sys = ActorSystem::create("projection-stop", config()).await.unwrap();
        let actor_ref = ProjectionActor::new(PathBuf::from("/bin/sleep"))
            .with_test_offline(true)
            .spawn(&sys, "projection-stop")
            .unwrap();
        let handle = actor_ref
            .request_projection(ProjectionSpec::defaults())
            .await
            .unwrap();
        actor_ref.stop_instance(handle.instance_id.clone()).await.unwrap();
        // Idempotent.
        actor_ref.stop_instance(handle.instance_id.clone()).await.unwrap();
        // Lookup of the handle should no longer return Some.
        let resolved = actor_ref.lookup_handle(handle.projection_id.clone()).await.unwrap();
        assert!(resolved.is_none());
        sys.terminate().await;
    }

    #[tokio::test]
    async fn projection_actor_pair_and_pin_offline() {
        let sys = ActorSystem::create("projection-pair", config()).await.unwrap();
        let actor_ref = ProjectionActor::new(PathBuf::from("/bin/sleep"))
            .with_test_offline(true)
            .spawn(&sys, "projection-pair")
            .unwrap();
        let handle = actor_ref
            .request_projection(ProjectionSpec::defaults())
            .await
            .unwrap();
        let client = ClientId::new();
        let ticket = actor_ref
            .pair_client(handle.instance_id.clone(), client.clone(), "pi-1".into())
            .await
            .unwrap();
        assert_eq!(ticket.client_id, client);
        actor_ref
            .submit_pin(
                handle.instance_id.clone(),
                client.clone(),
                "pi-1".into(),
                "1234".into(),
            )
            .await
            .unwrap();
        let pairings = actor_ref.known_pairings().await.unwrap();
        assert_eq!(pairings.len(), 1);
        assert_eq!(pairings[0].client_id, client);
        sys.terminate().await;
    }
}
