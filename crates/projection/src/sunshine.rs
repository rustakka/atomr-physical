//! Supervised Sunshine subprocess actor.
//!
//! A [`SunshineInstanceActor`] adapts a single `sunshine` binary into a
//! live atomr actor. The parent [`crate::ProjectionActor`] reserves a
//! [`PortWindow`] and renders the config file *before* spawning this
//! actor; once spawned, the runner owns the subprocess lifecycle:
//!
//! 1. `pre_start` writes the rendered config to a per-instance
//!    [`tempfile::TempDir`], spawns the child, splits stdout/stderr into
//!    log-pump tokio tasks, and hands the child off to an exit-watcher
//!    task that fires a [`SunshineInstanceMsg::ChildExited`] self-message
//!    on termination.
//! 2. [`SunshineInstanceMsg::Shutdown`] SIGTERMs the child by PID — the
//!    exit-watcher's `wait()` then returns and reports the exit code.
//! 3. `post_stop` issues a best-effort SIGTERM in case the actor is
//!    being torn down through the supervisor instead of `Shutdown`.
//!
//! The runner never panics on graceful child death: it just records the
//! exit code and marks the instance not-running. The atomr supervisor
//! restarts only on panic, so the policy is "supervisor-driven respawn
//! only on faults the runner could not handle itself".

use std::path::PathBuf;
use std::process::Stdio;
use std::time::Duration;

use async_trait::async_trait;
use atomr_core::actor::{Actor, ActorRef, ActorSystem, ActorSystemError, Context, Props};
use atomr_physical_core::{PhysicalError, Result, SunshineInstanceId};
use serde::{Deserialize, Serialize};
use tokio::io::{AsyncBufReadExt, BufReader};
use tokio::sync::oneshot;
use tracing::{debug, error, info, warn};

use crate::config_template::{render_config, write_instance_config, SunshineConfigParams};
use crate::ports::PortWindow;

/// Default ask timeout for [`SunshineInstanceRef`] helpers.
const ASK_TIMEOUT: Duration = Duration::from_secs(5);

/// Translate an atomr ask error into the physical-layer error taxonomy.
fn ask_to_physical(e: atomr_core::actor::AskError) -> PhysicalError {
    PhysicalError::Fault(format!("sunshine instance ask failed: {e:?}"))
}

/// Spec / builder for one supervised Sunshine instance.
///
/// Construct with [`SunshineInstanceActor::new`] and promote into a live
/// actor with [`spawn`](Self::spawn) or
/// [`spawn_under`](Self::spawn_under). The spec is cheap to clone: atomr
/// keeps the factory closure live for the supervisor's restart loop, so
/// every restart sees the same binary path, config, and extra args.
#[derive(Clone)]
pub struct SunshineInstanceActor {
    id: SunshineInstanceId,
    binary: PathBuf,
    config: SunshineConfigParams,
    /// Extra positional arguments appended after `--config <path>`. Used
    /// by tests passing `/bin/sleep` as a stand-in binary.
    extra_args: Vec<String>,
    /// Skip the `--config <path>` argument pair. Tests that point at a
    /// stand-in binary (e.g. `/bin/sleep`) enable this so the stand-in
    /// does not reject unknown flags.
    skip_config_arg: bool,
}

impl SunshineInstanceActor {
    /// Build a fresh spec. The supervisor is expected to reserve
    /// `config.port_window` *before* constructing this actor.
    pub fn new(id: SunshineInstanceId, binary: PathBuf, config: SunshineConfigParams) -> Self {
        Self {
            id,
            binary,
            config,
            extra_args: Vec::new(),
            skip_config_arg: false,
        }
    }

    /// Builder-style: append extra positional arguments to the child's
    /// command line. They are passed after `--config <path>`.
    pub fn with_extra_args(mut self, args: Vec<String>) -> Self {
        self.extra_args = args;
        self
    }

    /// Builder-style: skip the leading `--config <path>` argument pair.
    ///
    /// Production paths leave this `false`; tests that swap in a
    /// stand-in binary (e.g. `/bin/sleep`) enable it so the stand-in is
    /// not handed a flag it cannot parse.
    pub fn with_skip_config_arg(mut self, skip: bool) -> Self {
        self.skip_config_arg = skip;
        self
    }

    /// The supervisor's id for this instance.
    pub fn id(&self) -> &SunshineInstanceId {
        &self.id
    }

    /// The path to the `sunshine` binary that will be spawned.
    pub fn binary(&self) -> &PathBuf {
        &self.binary
    }

    /// The reserved port window — delegated from the underlying
    /// [`SunshineConfigParams`].
    pub fn port_window(&self) -> PortWindow {
        self.config.port_window
    }

    /// Promote into a supervised atomr actor at the top of an
    /// [`ActorSystem`].
    pub fn spawn(
        self,
        system: &ActorSystem,
        name: &str,
    ) -> std::result::Result<SunshineInstanceRef, ActorSystemError> {
        let (props, id, port_window) = self.into_runner_props();
        let actor_ref = system.actor_of(props, name)?;
        Ok(SunshineInstanceRef {
            inner: actor_ref,
            id,
            port_window,
        })
    }

    /// Promote into a supervised atomr actor as a child of `P`.
    ///
    /// Returns [`PhysicalError::Fault`] if atomr refuses the spawn (e.g.
    /// duplicate child name). The underlying `SpawnError` type isn't
    /// reachable through atomr-core 0.9.2's public surface, so we
    /// stringify it at the boundary.
    pub fn spawn_under<P: Actor>(self, ctx: &mut Context<P>, name: &str) -> Result<SunshineInstanceRef> {
        let (props, id, port_window) = self.into_runner_props();
        let actor_ref = ctx
            .spawn(props, name)
            .map_err(|e| PhysicalError::Fault(format!("sunshine instance child spawn failed: {e}")))?;
        Ok(SunshineInstanceRef {
            inner: actor_ref,
            id,
            port_window,
        })
    }

    fn into_runner_props(self) -> (Props<SunshineRunner>, SunshineInstanceId, PortWindow) {
        let id = self.id.clone();
        let port_window = self.config.port_window;
        let binary = self.binary;
        let config = self.config;
        let extra_args = self.extra_args;
        let skip_config_arg = self.skip_config_arg;
        let factory_id = id.clone();
        let props = Props::create(move || SunshineRunner {
            id: factory_id.clone(),
            binary: binary.clone(),
            config: config.clone(),
            extra_args: extra_args.clone(),
            skip_config_arg,
            child_pid: None,
            config_dir: None,
            last_exit_code: None,
            running: false,
            stop_requested: false,
        });
        (props, id, port_window)
    }
}

/// Mailbox of a live [`SunshineInstanceActor`].
///
/// Prefer the typed helpers on [`SunshineInstanceRef`] over reaching for
/// the variants directly — the helpers wrap the oneshot replies and the
/// ask timeout.
pub enum SunshineInstanceMsg {
    /// Drive a graceful shutdown: SIGTERM the child and reply when the
    /// signal has been delivered. The child is not awaited here; the
    /// exit-watcher records the exit code asynchronously.
    Shutdown {
        /// One-shot reply channel.
        reply: oneshot::Sender<Result<()>>,
    },
    /// Snapshot the instance's live state.
    GetSummary {
        /// One-shot reply channel.
        reply: oneshot::Sender<SunshineInstanceSummary>,
    },
    /// Internal self-message dispatched by the exit-watcher when the
    /// child terminates. Not intended for external callers.
    ChildExited {
        /// The child's exit code, if it produced one (signals → `None`).
        exit_code: Option<i32>,
    },
}

/// A snapshot of the supervisor's view of a live Sunshine instance.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SunshineInstanceSummary {
    /// The supervisor's id for this instance.
    pub id: SunshineInstanceId,
    /// The port window the instance was spawned against.
    pub port_window: PortWindow,
    /// The OS pid of the child, if it has been spawned.
    pub pid: Option<u32>,
    /// `true` while the child is alive.
    pub running: bool,
    /// The last observed exit code, if the child has terminated at
    /// least once.
    pub last_exit_code: Option<i32>,
}

/// A typed handle to a spawned [`SunshineInstanceActor`].
///
/// Cheap to clone; `tell`/`ask` go over the actor's mailbox.
#[derive(Clone)]
pub struct SunshineInstanceRef {
    inner: ActorRef<SunshineInstanceMsg>,
    id: SunshineInstanceId,
    port_window: PortWindow,
}

impl SunshineInstanceRef {
    /// The supervisor's id for this instance.
    pub fn id(&self) -> &SunshineInstanceId {
        &self.id
    }

    /// The port window the instance was spawned against.
    pub fn port_window(&self) -> PortWindow {
        self.port_window
    }

    /// The raw atomr actor reference.
    pub fn actor_ref(&self) -> &ActorRef<SunshineInstanceMsg> {
        &self.inner
    }

    /// Ask the actor to SIGTERM its child.
    ///
    /// Returns once the signal has been delivered (or after determining
    /// the child is no longer running). Calling twice is safe — the
    /// second call is a no-op.
    pub async fn shutdown(&self) -> Result<()> {
        self.inner
            .ask_with(|reply| SunshineInstanceMsg::Shutdown { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)?
    }

    /// Ask the actor for a snapshot of its current state.
    pub async fn summary(&self) -> Result<SunshineInstanceSummary> {
        self.inner
            .ask_with(|reply| SunshineInstanceMsg::GetSummary { reply }, ASK_TIMEOUT)
            .await
            .map_err(ask_to_physical)
    }
}

/// Internal Actor implementation backing a spawned
/// [`SunshineInstanceActor`].
///
/// The runner does **not** keep a [`tokio::process::Child`] alive in a
/// shared `Arc<Mutex<>>`: instead the child is moved into the
/// exit-watcher task, which owns the `wait()` future. To still permit
/// SIGTERM from `Shutdown` and `post_stop`, the child's pid is cached
/// in `child_pid` before the move.
struct SunshineRunner {
    id: SunshineInstanceId,
    binary: PathBuf,
    config: SunshineConfigParams,
    extra_args: Vec<String>,
    skip_config_arg: bool,
    /// The OS pid of the spawned child. `None` if `pre_start` has not
    /// run yet or the spawn failed.
    child_pid: Option<u32>,
    /// The tempdir guarding the rendered config; dropped at `post_stop`.
    config_dir: Option<tempfile::TempDir>,
    /// The last observed exit code (or `None` if the child died from a
    /// signal without a code).
    last_exit_code: Option<i32>,
    /// `true` while the child is alive — set in `pre_start`, cleared on
    /// `ChildExited`.
    running: bool,
    /// `true` once a [`SunshineInstanceMsg::Shutdown`] has been
    /// observed. Used to dampen the "unexpected exit" warning when the
    /// supervisor itself drove the teardown.
    stop_requested: bool,
}

impl SunshineRunner {
    /// Build the command line the runner will spawn.
    fn build_command(&self, cfg_path: &PathBuf) -> tokio::process::Command {
        let mut cmd = tokio::process::Command::new(&self.binary);
        if !self.skip_config_arg {
            cmd.arg("--config").arg(cfg_path);
        }
        for arg in &self.extra_args {
            cmd.arg(arg);
        }
        cmd.stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .kill_on_drop(true);
        cmd
    }

    /// Best-effort SIGTERM by pid; ignores `ESRCH` (no such process)
    /// and converts everything else into [`PhysicalError::Fault`].
    fn sigterm(&self, pid: u32) -> Result<()> {
        use nix::sys::signal::{kill, Signal};
        use nix::unistd::Pid;
        match kill(Pid::from_raw(pid as i32), Signal::SIGTERM) {
            Ok(()) => Ok(()),
            Err(nix::errno::Errno::ESRCH) => {
                debug!(instance = %self.id, pid, "sigterm: child already gone (ESRCH)");
                Ok(())
            }
            Err(e) => Err(PhysicalError::Fault(format!("sigterm pid={pid}: {e}"))),
        }
    }
}

#[async_trait]
impl Actor for SunshineRunner {
    type Msg = SunshineInstanceMsg;

    async fn pre_start(&mut self, ctx: &mut Context<Self>) {
        // 1. Render and write the config to a per-instance tempdir.
        let cfg_str = render_config(&self.config);
        let (cfg_path, tempdir) = match write_instance_config(&self.id, &cfg_str) {
            Ok(pair) => pair,
            Err(e) => {
                error!(instance = %self.id, error = %e, "failed to write sunshine config");
                return;
            }
        };
        self.config_dir = Some(tempdir);

        // 2. Spawn the subprocess.
        let mut cmd = self.build_command(&cfg_path);
        let mut child = match cmd.spawn() {
            Ok(c) => c,
            Err(e) => {
                error!(
                    instance = %self.id,
                    binary = %self.binary.display(),
                    error = %e,
                    "failed to spawn sunshine child"
                );
                return;
            }
        };

        // 3. Take stdout/stderr/pid *before* moving the child into the
        //    exit-watcher task. The pid is cached so SIGTERM can still
        //    reach the child after the move.
        let stdout = child.stdout.take();
        let stderr = child.stderr.take();
        let pid = child.id();
        self.child_pid = pid;
        self.running = true;

        info!(
            instance = %self.id,
            pid = ?pid,
            port = self.config.port_window.http_port(),
            "sunshine child spawned"
        );

        // 4. Log pump: stdout → tracing::info.
        if let Some(stdout) = stdout {
            let id = self.id.clone();
            tokio::spawn(async move {
                let mut reader = BufReader::new(stdout).lines();
                loop {
                    match reader.next_line().await {
                        Ok(Some(line)) => {
                            info!(instance = %id, stream = "stdout", "{}", line);
                        }
                        Ok(None) => break,
                        Err(e) => {
                            debug!(instance = %id, stream = "stdout", error = %e, "log pump read error");
                            break;
                        }
                    }
                }
                debug!(instance = %id, stream = "stdout", "log pump exiting");
            });
        }

        // 5. Log pump: stderr → tracing::warn.
        if let Some(stderr) = stderr {
            let id = self.id.clone();
            tokio::spawn(async move {
                let mut reader = BufReader::new(stderr).lines();
                loop {
                    match reader.next_line().await {
                        Ok(Some(line)) => {
                            warn!(instance = %id, stream = "stderr", "{}", line);
                        }
                        Ok(None) => break,
                        Err(e) => {
                            debug!(instance = %id, stream = "stderr", error = %e, "log pump read error");
                            break;
                        }
                    }
                }
                debug!(instance = %id, stream = "stderr", "log pump exiting");
            });
        }

        // 6. Exit watcher: owns the Child, calls wait(), self-messages
        //    on termination. The runner cannot wait() concurrently with
        //    the actor's handle loop, so ownership is moved here.
        let me = ctx.self_ref().clone();
        let id = self.id.clone();
        tokio::spawn(async move {
            let status = child.wait().await;
            let code = match status {
                Ok(s) => s.code(),
                Err(e) => {
                    warn!(instance = %id, error = %e, "child wait() failed");
                    None
                }
            };
            if !me.is_terminated() {
                me.tell(SunshineInstanceMsg::ChildExited { exit_code: code });
            } else {
                debug!(
                    instance = %id,
                    exit_code = ?code,
                    "child exited but actor already terminated; dropping ChildExited"
                );
            }
        });
    }

    async fn handle(&mut self, _ctx: &mut Context<Self>, msg: SunshineInstanceMsg) {
        match msg {
            SunshineInstanceMsg::Shutdown { reply } => {
                self.stop_requested = true;
                let result = if self.running {
                    if let Some(pid) = self.child_pid {
                        self.sigterm(pid)
                    } else {
                        debug!(instance = %self.id, "shutdown: running but no pid recorded");
                        Ok(())
                    }
                } else {
                    debug!(instance = %self.id, "shutdown: child already exited; no-op");
                    Ok(())
                };
                let _ = reply.send(result);
            }
            SunshineInstanceMsg::GetSummary { reply } => {
                let summary = SunshineInstanceSummary {
                    id: self.id.clone(),
                    port_window: self.config.port_window,
                    pid: self.child_pid,
                    running: self.running,
                    last_exit_code: self.last_exit_code,
                };
                let _ = reply.send(summary);
            }
            SunshineInstanceMsg::ChildExited { exit_code } => {
                self.running = false;
                self.last_exit_code = exit_code;
                if self.stop_requested {
                    info!(
                        instance = %self.id,
                        exit_code = ?exit_code,
                        "sunshine child exited after shutdown request"
                    );
                } else {
                    warn!(
                        instance = %self.id,
                        exit_code = ?exit_code,
                        "sunshine child exited unexpectedly (no shutdown requested)"
                    );
                }
                // We intentionally do not auto-stop the actor here: the
                // atomr supervisor restarts on panic, not on graceful
                // child death, and an unexpected exit may be something
                // the parent supervisor wants to inspect via
                // `GetSummary` before respawning.
            }
        }
    }

    async fn post_stop(&mut self, _ctx: &mut Context<Self>) {
        if self.running {
            if let Some(pid) = self.child_pid {
                if let Err(e) = self.sigterm(pid) {
                    warn!(
                        instance = %self.id,
                        pid,
                        error = %e,
                        "post_stop sigterm failed; child may linger until tokio kill_on_drop"
                    );
                }
            }
        }
        // Dropping `self.config_dir` here cleans the rendered config
        // off disk; happens implicitly as the runner is dropped.
        info!(instance = %self.id, "sunshine instance stopped");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use atomr_core::actor::ActorSystem;

    /// Build a minimal config for a stand-in binary. The port window is
    /// offset 0 — the tests never actually bind to it.
    fn test_config(instance: &SunshineInstanceId) -> SunshineConfigParams {
        SunshineConfigParams::defaults_for(
            instance.clone(),
            PortWindow::base(),
            PathBuf::from("/dev/dri/card0"),
        )
    }

    #[tokio::test]
    async fn sunshine_instance_with_sleep_stub() {
        let sys = ActorSystem::create("sunshine-test-1", atomr_config::Config::reference())
            .await
            .unwrap();
        let id = SunshineInstanceId::from("sun-test-1");
        let cfg = test_config(&id);
        let actor_ref = SunshineInstanceActor::new(id.clone(), PathBuf::from("/bin/sleep"), cfg)
            .with_extra_args(vec!["1".into()])
            .with_skip_config_arg(true)
            .spawn(&sys, "sunshine-1")
            .unwrap();

        // Give pre_start a moment to land — the actor system spawn is
        // synchronous but pre_start runs on the first scheduler tick.
        tokio::time::sleep(Duration::from_millis(150)).await;
        let snap = actor_ref.summary().await.unwrap();
        assert!(snap.pid.is_some(), "pid should be set after pre_start");
        assert!(snap.running, "should be running while sleep is alive");
        assert_eq!(snap.last_exit_code, None);

        // Sleep past the /bin/sleep duration and let the exit-watcher
        // self-message the actor.
        tokio::time::sleep(Duration::from_millis(1500)).await;
        let snap = actor_ref.summary().await.unwrap();
        assert!(!snap.running, "sleep should have exited by now");
        assert_eq!(snap.last_exit_code, Some(0));

        sys.terminate().await;
    }

    #[tokio::test]
    async fn sunshine_instance_shutdown_sigterms_running_child() {
        let sys = ActorSystem::create("sunshine-test-2", atomr_config::Config::reference())
            .await
            .unwrap();
        let id = SunshineInstanceId::from("sun-test-2");
        let cfg = test_config(&id);
        let actor_ref = SunshineInstanceActor::new(id.clone(), PathBuf::from("/bin/sleep"), cfg)
            .with_extra_args(vec!["30".into()])
            .with_skip_config_arg(true)
            .spawn(&sys, "sunshine-2")
            .unwrap();

        // Give pre_start a moment to land.
        tokio::time::sleep(Duration::from_millis(150)).await;
        let pre = actor_ref.summary().await.unwrap();
        assert!(pre.running, "sleep 30 should be running");

        actor_ref.shutdown().await.expect("shutdown ok");

        // The exit-watcher needs a moment to deliver ChildExited.
        tokio::time::sleep(Duration::from_millis(250)).await;
        let post = actor_ref.summary().await.unwrap();
        assert!(!post.running, "child should be gone after SIGTERM");

        sys.terminate().await;
    }

    #[tokio::test]
    async fn sunshine_instance_shutdown_idempotent() {
        let sys = ActorSystem::create("sunshine-test-3", atomr_config::Config::reference())
            .await
            .unwrap();
        let id = SunshineInstanceId::from("sun-test-3");
        let cfg = test_config(&id);
        let actor_ref = SunshineInstanceActor::new(id.clone(), PathBuf::from("/bin/sleep"), cfg)
            .with_extra_args(vec!["30".into()])
            .with_skip_config_arg(true)
            .spawn(&sys, "sunshine-3")
            .unwrap();

        tokio::time::sleep(Duration::from_millis(150)).await;
        actor_ref.shutdown().await.expect("first shutdown ok");
        tokio::time::sleep(Duration::from_millis(200)).await;
        actor_ref
            .shutdown()
            .await
            .expect("second shutdown should also succeed");

        sys.terminate().await;
    }

    #[test]
    fn builder_methods_round_trip() {
        let id = SunshineInstanceId::from("sun-builder");
        let cfg = test_config(&id);
        let actor = SunshineInstanceActor::new(id.clone(), PathBuf::from("/bin/sleep"), cfg.clone())
            .with_extra_args(vec!["1".into(), "2".into()])
            .with_skip_config_arg(true);
        assert_eq!(actor.id(), &id);
        assert_eq!(actor.binary(), &PathBuf::from("/bin/sleep"));
        assert_eq!(actor.port_window(), cfg.port_window);
    }
}
