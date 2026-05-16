//! End-to-end actor-surface integration tests for `atomr-physical-sdr`.
//!
//! These exercise [`SdrActor`] and [`SdrActorRef`] against the
//! hardware-free [`MockSdrDriver`] from the testkit so we cover the
//! full mailbox / broadcast pipeline without needing a HackRF plugged
//! in.

use std::sync::Arc;
use std::time::Duration;

use atomr_core::actor::ActorSystem;
use atomr_physical_sdr::{SdrActor, SdrParams};
use atomr_physical_testkit::{MockSdrDriver, MockWaveform};
use tokio::sync::broadcast::error::RecvError;
use tokio::time::timeout;

/// How long to wait for the broadcast channel to deliver the next
/// chunk in tests. The mock paces chunks aggressively (sub-millisecond
/// by default), so a multi-second budget should be plenty.
const RECV_TIMEOUT: Duration = Duration::from_secs(2);

#[tokio::test]
async fn snapshot_returns_requested_samples() {
    let mock = Arc::new(
        MockSdrDriver::new("snap-mock", MockWaveform::Zero)
            .with_chunk_samples(128)
            .with_chunk_interval(Duration::ZERO),
    );
    let actor = SdrActor::new(mock);
    let chunk = actor.snapshot(256).await.expect("snapshot succeeds");
    assert!(
        chunk.len_samples() >= 256,
        "expected >= 256 sample pairs, got {}",
        chunk.len_samples()
    );
}

#[tokio::test]
async fn spawned_actor_broadcasts_chunks() {
    let sys = ActorSystem::create("sdr-broadcast", atomr_config::Config::reference())
        .await
        .unwrap();
    let mock = Arc::new(
        MockSdrDriver::new("bcast-mock", MockWaveform::Ramp)
            .with_chunk_samples(64)
            .with_chunk_interval(Duration::ZERO),
    );
    let actor_ref = SdrActor::new(mock).spawn(&sys, "sdr-bcast").unwrap();
    let mut rx = actor_ref.subscribe();
    actor_ref.start_rx().await.unwrap();

    let mut seen: Vec<u64> = Vec::new();
    while seen.len() < 3 {
        match timeout(RECV_TIMEOUT, rx.recv()).await {
            Ok(Ok(chunk)) => seen.push(chunk.sequence),
            Ok(Err(RecvError::Lagged(_))) => continue,
            Ok(Err(RecvError::Closed)) => panic!("broadcast closed before 3 chunks"),
            Err(_) => panic!("timed out waiting for broadcast chunk"),
        }
    }
    for pair in seen.windows(2) {
        assert!(
            pair[1] > pair[0],
            "sequence numbers must be monotonic; saw {:?}",
            seen
        );
    }

    actor_ref.stop_rx().await.unwrap();
    sys.terminate().await;
}

#[tokio::test]
async fn tune_mid_stream_changes_centre() {
    let sys = ActorSystem::create("sdr-tune", atomr_config::Config::reference())
        .await
        .unwrap();
    // Use a small but non-zero chunk interval so the mock isn't
    // spinning so fast that the actor mailbox fills with RxChunk
    // messages ahead of our Tune ask. The runner is still serialised,
    // so eventually-consistent semantics hold either way; the cadence
    // just keeps the test budget reasonable.
    let mock = Arc::new(
        MockSdrDriver::new("tune-mock", MockWaveform::Zero)
            .with_chunk_samples(64)
            .with_chunk_interval(Duration::from_millis(1)),
    );
    let actor_ref = SdrActor::new(mock).spawn(&sys, "sdr-tune").unwrap();
    let mut rx = actor_ref.subscribe();
    actor_ref.start_rx().await.unwrap();

    // Capture the centre of the very first chunk we observe so we can
    // assert the tune actually moved it.
    let initial_centre = loop {
        match timeout(RECV_TIMEOUT, rx.recv()).await {
            Ok(Ok(chunk)) => break chunk.centre_hz,
            Ok(Err(RecvError::Lagged(_))) => continue,
            Ok(Err(RecvError::Closed)) => panic!("broadcast closed before first chunk"),
            Err(_) => panic!("timed out waiting for first chunk"),
        }
    };

    let new_params = SdrParams::default_rx().with_centre_hz(200_000_000);
    actor_ref.tune(new_params).await.unwrap();

    let mut found = false;
    for _ in 0..32 {
        match timeout(RECV_TIMEOUT, rx.recv()).await {
            Ok(Ok(chunk)) => {
                if chunk.centre_hz == 200_000_000 {
                    found = true;
                    break;
                }
            }
            Ok(Err(RecvError::Lagged(_))) => continue,
            Ok(Err(RecvError::Closed)) => break,
            Err(_) => panic!("timed out waiting for tuned chunk"),
        }
    }
    assert!(
        found,
        "tune did not propagate within 32 chunks (initial centre was {initial_centre})"
    );

    actor_ref.stop_rx().await.unwrap();
    sys.terminate().await;
}

#[tokio::test]
async fn transmit_via_mock_logs_payload() {
    let sys = ActorSystem::create("sdr-tx", atomr_config::Config::reference())
        .await
        .unwrap();
    let mock = Arc::new(MockSdrDriver::new("tx-mock", MockWaveform::Zero));
    let mock_for_assert = Arc::clone(&mock);
    let actor_ref = SdrActor::new(mock).spawn(&sys, "sdr-tx").unwrap();

    let payload: Arc<[i8]> = Arc::from(vec![1i8, 2, 3, 4]);
    actor_ref.transmit(payload).await.unwrap();

    let log = mock_for_assert.transmit_log();
    assert_eq!(log, vec![vec![1i8, 2, 3, 4]]);

    sys.terminate().await;
}

#[tokio::test]
async fn params_query_returns_initial() {
    let sys = ActorSystem::create("sdr-params", atomr_config::Config::reference())
        .await
        .unwrap();
    let initial = SdrParams::default_rx().with_centre_hz(433_000_000);
    let mock = Arc::new(MockSdrDriver::new("params-mock", MockWaveform::Zero));
    let actor_ref = SdrActor::new(mock)
        .with_params(initial.clone())
        .spawn(&sys, "sdr-params")
        .unwrap();

    let observed = actor_ref.params().await.unwrap();
    assert_eq!(observed.centre_hz, initial.centre_hz);

    sys.terminate().await;
}

#[tokio::test]
async fn health_check_succeeds() {
    let sys = ActorSystem::create("sdr-health", atomr_config::Config::reference())
        .await
        .unwrap();
    let mock = Arc::new(MockSdrDriver::new("health-mock", MockWaveform::Zero));
    let actor_ref = SdrActor::new(mock).spawn(&sys, "sdr-health").unwrap();

    actor_ref.health().await.unwrap();

    sys.terminate().await;
}

#[tokio::test]
async fn stop_rx_is_idempotent() {
    let sys = ActorSystem::create("sdr-stop-idem", atomr_config::Config::reference())
        .await
        .unwrap();
    let mock = Arc::new(
        MockSdrDriver::new("stop-mock", MockWaveform::Zero)
            .with_chunk_samples(32)
            .with_chunk_interval(Duration::ZERO),
    );
    let actor_ref = SdrActor::new(mock).spawn(&sys, "sdr-stop").unwrap();

    actor_ref.start_rx().await.unwrap();
    actor_ref.stop_rx().await.unwrap();
    actor_ref.stop_rx().await.unwrap();

    sys.terminate().await;
}
