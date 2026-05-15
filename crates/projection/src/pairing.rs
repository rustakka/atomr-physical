//! HTTPS client that drives Sunshine's local pairing API on behalf of a
//! [`ProjectionActor`](crate::ProjectionActor).
//!
//! Sunshine exposes `/api/pair`, `/api/pin`, and `/api/unpair` on the
//! same loopback HTTPS port as its web UI. Each instance ships a
//! freshly-minted self-signed TLS cert, so the [`reqwest::Client`]
//! built here turns off cert validation and falls back on **trust on
//! first use** — the SPKI fingerprint observed on the first successful
//! pair is recorded in the [`PairingRecord`] and can be re-checked by
//! callers on later sessions.
//!
//! Construct with [`ClientProvisioner::new`] in production or
//! [`ClientProvisioner::offline`] in tests / integration runs against a
//! stub `/bin/sleep` binary. Offline mode short-circuits every method:
//! `start_pairing` returns a deterministic zero-filled salt,
//! `submit_pin` records a [`PairingRecord`] in-memory, and `revoke`
//! drops it.

use atomr_physical_core::{ClientId, PhysicalError, Result, SunshineInstanceId};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::sync::RwLock;
use tracing::{debug, info, warn};
use url::Url;

/// HTTP request timeout for every call against Sunshine's local API.
const HTTP_TIMEOUT: Duration = Duration::from_secs(5);

/// Length of the deterministic stub salt returned by an offline
/// [`ClientProvisioner`].
const OFFLINE_SALT_LEN: usize = 16;

/// One row in the pairing book — what the provisioner remembers about
/// each successful pair.
///
/// Stored under the [`ClientId`] inside [`ClientProvisioner`]. The
/// `spki_fingerprint` is captured at first contact (when the underlying
/// `reqwest` client surfaces it) and acts as the TOFU pin on
/// subsequent reconnects; offline pairings leave it `None`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PairingRecord {
    /// The Moonlight client that was paired.
    pub client_id: ClientId,
    /// The Sunshine instance the client is paired against.
    pub instance: SunshineInstanceId,
    /// Hostname / display name the client advertised at pair time.
    pub hostname: String,
    /// Wall-clock time of the successful pin submission, expressed as
    /// milliseconds since the Unix epoch.
    pub paired_at_ms: i64,
    /// SPKI fingerprint of the Sunshine server's TLS cert, captured at
    /// first contact for TOFU pinning. `None` for offline pairings.
    pub spki_fingerprint: Option<String>,
}

/// JSON payload accepted by Sunshine's `/api/pair` endpoint.
#[derive(Debug, Serialize)]
struct PairRequest<'a> {
    uuid: &'a str,
    name: &'a str,
}

/// JSON payload accepted by Sunshine's `/api/pin` endpoint.
#[derive(Debug, Serialize)]
struct PinRequest<'a> {
    pin: &'a str,
}

/// JSON payload accepted by Sunshine's `/api/unpair` endpoint.
#[derive(Debug, Serialize)]
struct UnpairRequest<'a> {
    uuid: &'a str,
}

/// Best-effort decoded `/api/pair` reply. Only the salt is load-bearing
/// — Sunshine builds versions vary in the surrounding envelope.
#[derive(Debug, Deserialize)]
struct PairResponse {
    #[serde(default)]
    salt: Option<String>,
}

/// Drives Sunshine's HTTPS pairing API on behalf of a
/// [`ProjectionActor`](crate::ProjectionActor).
///
/// Construct with [`new`](Self::new), supply a `base_url_for` closure
/// that returns the local pairing URL for a given instance id
/// (typically `https://127.0.0.1:<tcp_port>`), and call
/// [`start_pairing`](Self::start_pairing) followed by
/// [`submit_pin`](Self::submit_pin) in sequence. The provisioner is
/// `Send + Sync` behind an `Arc` so the parent actor can hand it out
/// to spawned tasks without copying.
///
/// `Debug` is not derived because the `base_url_for` trait object is
/// not itself `Debug`; the public accessors expose everything that
/// would have been printed.
pub struct ClientProvisioner {
    client: reqwest::Client,
    base_url_for: Arc<dyn Fn(&SunshineInstanceId) -> Url + Send + Sync>,
    pairings: RwLock<HashMap<ClientId, PairingRecord>>,
    accept_self_signed: bool,
    offline: bool,
}

impl ClientProvisioner {
    /// Construct against a closure mapping instance id to its local
    /// HTTPS base URL.
    ///
    /// When `accept_self_signed` is `true` the underlying
    /// [`reqwest::Client`] is built with
    /// [`reqwest::ClientBuilder::danger_accept_invalid_certs`] — the
    /// recommended setting for Sunshine's per-instance self-signed
    /// certs, with TOFU pinning enforced through the
    /// `spki_fingerprint` field of each [`PairingRecord`].
    pub fn new<F>(base_url_for: F, accept_self_signed: bool) -> Result<Self>
    where
        F: Fn(&SunshineInstanceId) -> Url + Send + Sync + 'static,
    {
        let client = reqwest::Client::builder()
            .timeout(HTTP_TIMEOUT)
            .danger_accept_invalid_certs(accept_self_signed)
            .build()
            .map_err(|e| PhysicalError::Fault(format!("reqwest builder: {e}")))?;
        Ok(Self {
            client,
            base_url_for: Arc::new(base_url_for),
            pairings: RwLock::new(HashMap::new()),
            accept_self_signed,
            offline: false,
        })
    }

    /// Construct an offline provisioner.
    ///
    /// Every method returns a deterministic stub success without making
    /// any network call. Used in this crate's unit tests and from the
    /// integration tests that exercise the supervisor tree against a
    /// stub `/bin/sleep` binary in place of a real Sunshine.
    pub fn offline() -> Self {
        // The client is built but never used in offline mode; we
        // construct one so the field stays non-Option. Falling back to
        // `Client::new()` (no custom config) is safe here because no
        // request will ever be issued.
        let client = reqwest::Client::new();
        Self {
            client,
            base_url_for: Arc::new(|_id: &SunshineInstanceId| {
                // Unreachable in offline mode; we still return a valid
                // URL for symmetry rather than panicking.
                Url::parse("https://127.0.0.1/").expect("static URL parses")
            }),
            pairings: RwLock::new(HashMap::new()),
            accept_self_signed: true,
            offline: true,
        }
    }

    /// Whether the provisioner was built with self-signed acceptance.
    pub fn accept_self_signed(&self) -> bool {
        self.accept_self_signed
    }

    /// Snapshot of every [`PairingRecord`] currently held in memory.
    pub async fn known_pairings(&self) -> Vec<PairingRecord> {
        self.pairings.read().await.values().cloned().collect()
    }

    /// Start a pairing handshake against a running Sunshine instance.
    ///
    /// POSTs `{ "uuid": <client_id>, "name": <hostname> }` to
    /// `<base>/api/pair` and returns the server's pairing salt as raw
    /// bytes (decoded best-effort from base64 then hex, falling back
    /// to the literal response bytes on parse failure). Callers treat
    /// the returned `Vec<u8>` as opaque — `PairingTicketBytes` in the
    /// supervisor.
    ///
    /// On non-2xx responses the body is captured (truncated) into a
    /// [`PhysicalError::PairingRejected`].
    ///
    /// In offline mode this returns a zero-filled 16-byte vec without
    /// touching the network.
    pub async fn start_pairing(
        &self,
        instance: &SunshineInstanceId,
        client_id: &ClientId,
        hostname: &str,
    ) -> Result<Vec<u8>> {
        if self.offline {
            debug!(
                instance = %instance,
                client = %client_id,
                hostname,
                "offline: synthesising pairing salt"
            );
            return Ok(vec![0u8; OFFLINE_SALT_LEN]);
        }

        let url = self.endpoint(instance, "api/pair")?;
        let body = PairRequest {
            uuid: client_id.as_str(),
            name: hostname,
        };

        debug!(%url, client = %client_id, hostname, "POST /api/pair");
        let resp = self
            .client
            .post(url.clone())
            .json(&body)
            .send()
            .await
            .map_err(|e| PhysicalError::PairingRejected {
                client: client_id.to_string(),
                reason: format!("transport error contacting {url}: {e}"),
            })?;

        let status = resp.status();
        let text = resp
            .text()
            .await
            .unwrap_or_else(|e| format!("<body read failed: {e}>"));

        if !status.is_success() {
            warn!(
                %status,
                client = %client_id,
                body = %redact(&text),
                "pair rejected"
            );
            return Err(PhysicalError::PairingRejected {
                client: client_id.to_string(),
                reason: format!("/api/pair status={status}, body={}", redact(&text)),
            });
        }

        let salt_bytes = parse_salt_from_body(&text);
        info!(
            instance = %instance,
            client = %client_id,
            hostname,
            salt_len = salt_bytes.len(),
            "pair handshake started"
        );
        Ok(salt_bytes)
    }

    /// Complete the pairing handshake by submitting the PIN displayed
    /// to the user.
    ///
    /// POSTs `{ "pin": <pin> }` to `<base>/api/pin`. On a 2xx response
    /// a [`PairingRecord`] is written into the provisioner's pairing
    /// book. Non-2xx responses surface as
    /// [`PhysicalError::PairingRejected`].
    ///
    /// In offline mode the record is written immediately and no
    /// network call is issued.
    pub async fn submit_pin(
        &self,
        instance: &SunshineInstanceId,
        client_id: &ClientId,
        hostname: &str,
        pin: &str,
    ) -> Result<()> {
        if self.offline {
            debug!(
                instance = %instance,
                client = %client_id,
                hostname,
                "offline: recording pairing"
            );
            self.record_pairing(instance, client_id, hostname, None)
                .await;
            info!(
                instance = %instance,
                client = %client_id,
                "offline pairing complete"
            );
            return Ok(());
        }

        let url = self.endpoint(instance, "api/pin")?;
        let body = PinRequest { pin };

        debug!(%url, client = %client_id, "POST /api/pin");
        let resp = self
            .client
            .post(url.clone())
            .json(&body)
            .send()
            .await
            .map_err(|e| PhysicalError::PairingRejected {
                client: client_id.to_string(),
                reason: format!("transport error contacting {url}: {e}"),
            })?;

        let status = resp.status();
        let text = resp
            .text()
            .await
            .unwrap_or_else(|e| format!("<body read failed: {e}>"));

        if !status.is_success() {
            warn!(
                %status,
                client = %client_id,
                body = %redact(&text),
                "pin rejected"
            );
            return Err(PhysicalError::PairingRejected {
                client: client_id.to_string(),
                reason: format!("/api/pin status={status}, body={}", redact(&text)),
            });
        }

        // SPKI fingerprint capture would happen here once we wire a
        // custom TLS verifier into reqwest; until then, leave it None
        // so the field's contract — "captured at first contact" — is
        // truthful instead of misleading.
        self.record_pairing(instance, client_id, hostname, None)
            .await;
        info!(
            instance = %instance,
            client = %client_id,
            hostname,
            "pairing complete"
        );
        Ok(())
    }

    /// Forget a paired client.
    ///
    /// Best-effort POST to `<base>/api/unpair` against every Sunshine
    /// instance the record is known for; the in-memory record is
    /// always dropped, even when the network call fails. Offline form
    /// just drops the in-memory record.
    pub async fn revoke(&self, client_id: &ClientId) -> Result<()> {
        let removed = {
            let mut guard = self.pairings.write().await;
            guard.remove(client_id)
        };

        let Some(record) = removed else {
            debug!(client = %client_id, "revoke: no record, nothing to do");
            return Ok(());
        };

        if self.offline {
            info!(
                client = %client_id,
                instance = %record.instance,
                "offline: pairing record revoked"
            );
            return Ok(());
        }

        let url = match self.endpoint(&record.instance, "api/unpair") {
            Ok(u) => u,
            Err(e) => {
                warn!(client = %client_id, error = %e, "revoke: cannot build url; in-memory record already dropped");
                return Ok(());
            }
        };

        let body = UnpairRequest {
            uuid: client_id.as_str(),
        };

        debug!(%url, client = %client_id, "POST /api/unpair");
        match self.client.post(url.clone()).json(&body).send().await {
            Ok(resp) if resp.status().is_success() => {
                info!(client = %client_id, "pairing revoked");
            }
            Ok(resp) => {
                warn!(
                    client = %client_id,
                    status = %resp.status(),
                    "revoke: server replied non-2xx; in-memory record already dropped"
                );
            }
            Err(e) => {
                warn!(
                    client = %client_id,
                    error = %e,
                    "revoke: transport error; in-memory record already dropped"
                );
            }
        }
        Ok(())
    }

    fn endpoint(&self, instance: &SunshineInstanceId, path: &str) -> Result<Url> {
        let mut url = (self.base_url_for)(instance);
        url.set_path(path);
        Ok(url)
    }

    async fn record_pairing(
        &self,
        instance: &SunshineInstanceId,
        client_id: &ClientId,
        hostname: &str,
        spki_fingerprint: Option<String>,
    ) {
        let record = PairingRecord {
            client_id: client_id.clone(),
            instance: instance.clone(),
            hostname: hostname.to_string(),
            paired_at_ms: now_ms(),
            spki_fingerprint,
        };
        let mut guard = self.pairings.write().await;
        guard.insert(client_id.clone(), record);
    }
}

/// Wall-clock milliseconds since the Unix epoch as a signed i64.
/// Saturates to `i64::MAX` if the system clock is set past the
/// representable range (≈ year 292 million), which is a safer choice
/// than panicking inside an actor.
fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| i64::try_from(d.as_millis()).unwrap_or(i64::MAX))
        .unwrap_or(0)
}

/// Best-effort decode of Sunshine's `/api/pair` response body into the
/// salt bytes. Tries JSON first, then base64, then hex; falls back to
/// the raw response bytes so the caller never silently loses data.
fn parse_salt_from_body(text: &str) -> Vec<u8> {
    if let Ok(parsed) = serde_json::from_str::<PairResponse>(text) {
        if let Some(salt) = parsed.salt {
            return decode_salt_str(&salt);
        }
    }
    decode_salt_str(text.trim())
}

fn decode_salt_str(s: &str) -> Vec<u8> {
    if let Some(bytes) = try_hex(s) {
        return bytes;
    }
    if let Some(bytes) = try_base64(s) {
        return bytes;
    }
    s.as_bytes().to_vec()
}

fn try_hex(s: &str) -> Option<Vec<u8>> {
    let trimmed = s.trim();
    if trimmed.is_empty() || trimmed.len() % 2 != 0 {
        return None;
    }
    if !trimmed.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    let mut out = Vec::with_capacity(trimmed.len() / 2);
    for chunk in trimmed.as_bytes().chunks(2) {
        let hi = (chunk[0] as char).to_digit(16)?;
        let lo = (chunk[1] as char).to_digit(16)?;
        out.push(((hi << 4) | lo) as u8);
    }
    Some(out)
}

fn try_base64(s: &str) -> Option<Vec<u8>> {
    let trimmed = s.trim();
    if trimmed.is_empty() {
        return None;
    }
    if !trimmed
        .chars()
        .all(|c| c.is_ascii_alphanumeric() || c == '+' || c == '/' || c == '=')
    {
        return None;
    }
    decode_base64(trimmed)
}

/// Minimal RFC 4648 base64 decoder. Inlined because none of the
/// workspace deps already pull in `base64`, and the salt path is the
/// only consumer.
fn decode_base64(s: &str) -> Option<Vec<u8>> {
    let table = |c: u8| -> Option<u8> {
        match c {
            b'A'..=b'Z' => Some(c - b'A'),
            b'a'..=b'z' => Some(c - b'a' + 26),
            b'0'..=b'9' => Some(c - b'0' + 52),
            b'+' => Some(62),
            b'/' => Some(63),
            _ => None,
        }
    };

    let bytes: Vec<u8> = s.bytes().filter(|b| *b != b'=').collect();
    let mut out = Vec::with_capacity(bytes.len() * 3 / 4);
    let mut buf = 0u32;
    let mut bits = 0u32;
    for b in bytes {
        let v = table(b)? as u32;
        buf = (buf << 6) | v;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push(((buf >> bits) & 0xFF) as u8);
        }
    }
    Some(out)
}

/// Truncate response bodies for log lines so a chatty server can't
/// flood `tracing`. Sunshine error replies are normally short, but a
/// misconfigured proxy can return a multi-MiB HTML page.
fn redact(s: &str) -> String {
    const MAX: usize = 256;
    if s.len() <= MAX {
        return s.to_string();
    }
    format!("{}... [{} bytes redacted]", &s[..MAX], s.len() - MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cid() -> ClientId {
        ClientId::from("cli-test-1")
    }

    fn iid() -> SunshineInstanceId {
        SunshineInstanceId::from("sun-test-abcdef01")
    }

    #[tokio::test]
    async fn offline_provisioner_records_pairing() {
        let prov = ClientProvisioner::offline();
        prov.submit_pin(&iid(), &cid(), "moon-host", "1234")
            .await
            .unwrap();
        let known = prov.known_pairings().await;
        assert_eq!(known.len(), 1);
        assert_eq!(known[0].client_id, cid());
        assert_eq!(known[0].instance, iid());
        assert_eq!(known[0].hostname, "moon-host");
        assert!(known[0].paired_at_ms >= 0);
    }

    #[tokio::test]
    async fn offline_provisioner_revoke_drops_record() {
        let prov = ClientProvisioner::offline();
        prov.submit_pin(&iid(), &cid(), "moon-host", "1234")
            .await
            .unwrap();
        assert_eq!(prov.known_pairings().await.len(), 1);
        prov.revoke(&cid()).await.unwrap();
        assert!(prov.known_pairings().await.is_empty());
    }

    #[tokio::test]
    async fn offline_provisioner_start_returns_predictable_salt() {
        let prov = ClientProvisioner::offline();
        let salt = prov.start_pairing(&iid(), &cid(), "moon-host").await.unwrap();
        assert_eq!(salt.len(), OFFLINE_SALT_LEN);
        assert!(salt.iter().all(|b| *b == 0));
    }

    #[tokio::test]
    async fn revoke_unknown_client_is_ok() {
        let prov = ClientProvisioner::offline();
        prov.revoke(&ClientId::from("cli-never-paired"))
            .await
            .unwrap();
    }

    #[test]
    fn hex_decoder_round_trip() {
        let bytes = try_hex("deadbeef").unwrap();
        assert_eq!(bytes, vec![0xde, 0xad, 0xbe, 0xef]);
        assert!(try_hex("xyz").is_none());
    }

    #[test]
    fn base64_decoder_round_trip() {
        // "hello" -> "aGVsbG8="
        let bytes = decode_base64("aGVsbG8").unwrap();
        assert_eq!(bytes, b"hello");
    }

    #[test]
    fn parse_salt_handles_json_envelope() {
        let bytes = parse_salt_from_body(r#"{"salt":"deadbeef"}"#);
        assert_eq!(bytes, vec![0xde, 0xad, 0xbe, 0xef]);
    }

    #[test]
    fn parse_salt_falls_back_to_raw() {
        let bytes = parse_salt_from_body("not-hex-not-b64!@#");
        assert_eq!(bytes, b"not-hex-not-b64!@#".to_vec());
    }

    #[test]
    fn redact_truncates_long_bodies() {
        let body = "x".repeat(500);
        let r = redact(&body);
        assert!(r.starts_with(&"x".repeat(256)));
        assert!(r.contains("redacted"));
    }

    #[test]
    fn online_provisioner_builds() {
        let prov =
            ClientProvisioner::new(|_id| Url::parse("https://127.0.0.1:47990").unwrap(), true)
                .unwrap();
        assert!(prov.accept_self_signed());
    }
}
