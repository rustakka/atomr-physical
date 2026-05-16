//! End-to-end test for the `sigmf` feature: spawn an actor, subscribe
//! its broadcast stream into a [`SigmfWriter`], and verify the
//! resulting `.sigmf-data` / `.sigmf-meta` pair on disk.

#![cfg(feature = "sigmf")]

use std::sync::Arc;
use std::time::Duration;

use atomr_core::actor::ActorSystem;
use atomr_physical_sdr::{persist_until_eos, PersistConfig, SdrActor, SigmfMeta, SigmfWriter};
use atomr_physical_testkit::{MockSdrDriver, MockWaveform};
use tempfile::tempdir;

#[tokio::test]
async fn persist_until_eos_writes_data_and_meta() {
    let sys = ActorSystem::create("sdr-sigmf", atomr_config::Config::reference())
        .await
        .unwrap();
    let mock = Arc::new(
        MockSdrDriver::new("sigmf-mock", MockWaveform::Ramp)
            .with_chunk_samples(128)
            .with_chunk_interval(Duration::from_millis(1)),
    );
    let actor_ref = SdrActor::new(mock).spawn(&sys, "sdr-sigmf").unwrap();
    let rx = actor_ref.subscribe();
    actor_ref.start_rx().await.unwrap();

    let dir = tempdir().unwrap();
    let base = dir.path().join("test");
    let writer = SigmfWriter::open(PersistConfig::at(base.clone()))
        .await
        .unwrap();

    let writer_task = tokio::spawn(persist_until_eos(rx, writer));

    // Let some chunks flow into the writer.
    tokio::time::sleep(Duration::from_millis(200)).await;

    // Stop the stream and tear down the actor system so the broadcast
    // channel closes — that's what kicks `persist_until_eos` out of
    // its recv loop.
    actor_ref.stop_rx().await.unwrap();
    drop(actor_ref);
    sys.terminate().await;

    let writer = tokio::time::timeout(Duration::from_secs(5), writer_task)
        .await
        .expect("writer task did not finish in time")
        .expect("writer task panicked")
        .expect("writer returned error");

    // Both final files must exist now that `persist_until_eos` ran
    // `close()` on the writer.
    let data_path = writer.data_path().to_owned();
    let meta_path = writer.meta_path().to_owned();
    assert!(
        data_path.exists(),
        "expected sigmf-data at {}",
        data_path.display()
    );
    assert!(
        meta_path.exists(),
        "expected sigmf-meta at {}",
        meta_path.display()
    );

    let data_len = std::fs::metadata(&data_path).unwrap().len();
    assert!(
        data_len > 0,
        "expected non-empty sigmf-data, got {data_len} bytes"
    );

    let meta_text = std::fs::read_to_string(&meta_path).unwrap();
    let parsed: SigmfMeta = serde_json::from_str(&meta_text).expect("parse sigmf-meta JSON");
    assert!(
        !parsed.captures.is_empty(),
        "expected at least one capture entry in {meta_text}"
    );
}
