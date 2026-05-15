//! End-to-end integration tests for [`ProjectionActor`].
//!
//! These tests boot a live `ActorSystem`, exercise the full supervisor
//! tree (port allocator, vkms-shim, sunshine-shim, pairing, mDNS) in
//! the offline pathway, and assert the observable state — they do not
//! require root, vkms, or a real Sunshine binary.

use std::path::PathBuf;
use std::time::Duration;

use atomr_core::actor::ActorSystem;
use atomr_physical_core::ClientId;
use atomr_physical_projection::{ProjectionActor, ProjectionSpec};

fn config() -> atomr_config::Config {
    atomr_config::Config::reference()
}

#[tokio::test]
async fn two_projections_share_one_supervisor() {
    let sys = ActorSystem::create("itest-projection-two", config()).await.unwrap();
    let actor_ref = ProjectionActor::new(PathBuf::from("/bin/sleep"))
        .with_test_offline(true)
        .with_mdns_host_label("atomr-itest")
        .spawn(&sys, "projection-itest-two")
        .unwrap();

    let h1 = actor_ref.request_projection(ProjectionSpec::defaults()).await.unwrap();
    let h2 = actor_ref.request_projection(ProjectionSpec::defaults()).await.unwrap();

    // Distinct ports, distinct displays, distinct instances.
    assert_ne!(h1.instance_id, h2.instance_id);
    assert_ne!(h1.display_id, h2.display_id);
    assert_ne!(h1.port_window.offset, h2.port_window.offset);
    assert_ne!(h1.mdns_service, h2.mdns_service);

    // Both should appear in the live snapshot.
    tokio::time::sleep(Duration::from_millis(200)).await;
    let summaries = actor_ref.list_instances().await.unwrap();
    assert_eq!(summaries.len(), 2);

    // Tearing one down leaves the other.
    actor_ref.stop_instance(h1.instance_id.clone()).await.unwrap();
    let after = actor_ref.list_instances().await.unwrap();
    assert_eq!(after.len(), 1);
    assert_eq!(after[0].id, h2.instance_id);

    sys.terminate().await;
}

#[tokio::test]
async fn pair_and_revoke_offline_pipeline() {
    let sys = ActorSystem::create("itest-projection-pair", config()).await.unwrap();
    let actor_ref = ProjectionActor::new(PathBuf::from("/bin/sleep"))
        .with_test_offline(true)
        .spawn(&sys, "projection-itest-pair")
        .unwrap();
    let handle = actor_ref.request_projection(ProjectionSpec::defaults()).await.unwrap();

    // Pair two clients against the same instance.
    let c1 = ClientId::new();
    let c2 = ClientId::new();
    let t1 = actor_ref
        .pair_client(handle.instance_id.clone(), c1.clone(), "node-1".into())
        .await
        .unwrap();
    let t2 = actor_ref
        .pair_client(handle.instance_id.clone(), c2.clone(), "node-2".into())
        .await
        .unwrap();
    assert_eq!(t1.client_id, c1);
    assert_eq!(t2.client_id, c2);

    actor_ref
        .submit_pin(handle.instance_id.clone(), c1.clone(), "node-1".into(), "0001".into())
        .await
        .unwrap();
    actor_ref
        .submit_pin(handle.instance_id.clone(), c2.clone(), "node-2".into(), "0002".into())
        .await
        .unwrap();

    let pairings = actor_ref.known_pairings().await.unwrap();
    assert_eq!(pairings.len(), 2);

    actor_ref.stop_instance(handle.instance_id).await.unwrap();
    sys.terminate().await;
}

#[tokio::test]
async fn lookup_handle_finds_only_live_projections() {
    let sys = ActorSystem::create("itest-projection-lookup", config()).await.unwrap();
    let actor_ref = ProjectionActor::new(PathBuf::from("/bin/sleep"))
        .with_test_offline(true)
        .spawn(&sys, "projection-itest-lookup")
        .unwrap();
    let handle = actor_ref.request_projection(ProjectionSpec::defaults()).await.unwrap();
    let lookup = actor_ref.lookup_handle(handle.projection_id.clone()).await.unwrap();
    assert!(lookup.is_some());
    actor_ref.stop_instance(handle.instance_id).await.unwrap();
    let lookup_after = actor_ref.lookup_handle(handle.projection_id).await.unwrap();
    assert!(lookup_after.is_none());
    sys.terminate().await;
}
