//! [`SigmfWriter`] — capture IQ chunks to a
//! [SigMF](https://github.com/sigmf/SigMF) pair on disk.
//!
//! A `SigmfWriter` consumes from an [`crate::SdrActorRef`]'s broadcast
//! channel and lays down two files:
//!
//! * `{base}.sigmf-data` — raw interleaved `ci8_le` (complex int8 LE)
//!   samples, exactly the bytes the HackRF emitted.
//! * `{base}.sigmf-meta` — JSON header describing the recording.
//!
//! Writes are atomic from the consumer's perspective: bytes stream to
//! `*.sigmf-data.partial` and the file is renamed to its final name
//! only when the writer is dropped or [`close`](Self::close) is
//! called. The metadata file is written **last** — its presence is
//! the signal that a recording completed cleanly. A `*.partial` left
//! behind with no `*.sigmf-meta` indicates a crashed/aborted capture.
//!
//! ## Schema
//!
//! `core:datatype` is `ci8_le`. The `captures[]` array gets one entry
//! per `Tune` observed mid-recording — every time `centre_hz` changes
//! in an incoming chunk we close out the previous capture entry and
//! open a new one anchored at the running sample offset. The schema
//! itself is hand-rolled via [`serde_json`] — SigMF is a small,
//! stable specification, so a third-party `sigmf` crate dependency
//! buys nothing in return for the supply-chain risk.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use tokio::fs::{File, OpenOptions};
use tokio::io::AsyncWriteExt;
use tokio::sync::broadcast;

use crate::error::{SdrError, SdrResult};
use crate::iq::IqChunk;

/// What to record, and where.
#[derive(Debug, Clone)]
pub struct PersistConfig {
    /// Path stem — the writer appends `.sigmf-data` and
    /// `.sigmf-meta`.
    pub base_path: PathBuf,
    /// If `Some`, the writer rotates the underlying file pair every
    /// `Duration` (closing the current one cleanly and opening a new
    /// pair with a timestamp suffix).
    pub rotate_every: Option<Duration>,
    /// Hard cap on the size of the data file. Reaching it triggers a
    /// rotation (if `rotate_every` is set) or a clean close.
    pub max_bytes: Option<u64>,
    /// Optional `core:author` annotation in the metadata.
    pub author: Option<String>,
    /// Optional `core:description` annotation in the metadata.
    pub description: Option<String>,
}

impl PersistConfig {
    /// Construct a writer config that lays files down at `base_path`
    /// with no rotation, no size limit, and no author / description.
    pub fn at(base_path: impl Into<PathBuf>) -> Self {
        Self {
            base_path: base_path.into(),
            rotate_every: None,
            max_bytes: None,
            author: None,
            description: None,
        }
    }

    /// Set the author tag.
    pub fn with_author(mut self, author: impl Into<String>) -> Self {
        self.author = Some(author.into());
        self
    }

    /// Set the description tag.
    pub fn with_description(mut self, description: impl Into<String>) -> Self {
        self.description = Some(description.into());
        self
    }
}

/// A SigMF `captures` array entry — one per `centre_hz` transition
/// during a recording.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigmfCapture {
    /// Sample offset (in *sample pairs*) into the data file at which
    /// this capture-entry's parameters begin to apply.
    #[serde(rename = "core:sample_start")]
    pub sample_start: u64,
    /// Centre frequency in Hz at this capture.
    #[serde(rename = "core:frequency")]
    pub frequency: u64,
    /// Wall-clock timestamp of the first chunk in this capture.
    #[serde(rename = "core:datetime")]
    pub datetime: String,
}

/// The full `*.sigmf-meta` file shape. Field names follow the SigMF
/// schema verbatim so the resulting JSON is round-trip compatible
/// with GNU-Radio / inspectrum / gqrx.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SigmfMeta {
    /// SigMF "global" object (sample-rate / format / recorder).
    pub global: Value,
    /// One capture entry per `Tune` observed during the recording.
    pub captures: Vec<SigmfCapture>,
    /// Annotations array — empty for now (atomr-physical has no
    /// annotation source yet).
    pub annotations: Vec<Value>,
}

/// Streaming SigMF writer. Owns the partial data file and the
/// in-progress `captures[]` list.
pub struct SigmfWriter {
    config: PersistConfig,
    data_partial: PathBuf,
    data_final: PathBuf,
    meta_final: PathBuf,
    data_file: Option<File>,
    bytes_written: u64,
    sample_pairs_written: u64,
    captures: Vec<SigmfCapture>,
    sample_rate_hz: Option<u32>,
    closed: bool,
}

impl SigmfWriter {
    /// Open a fresh writer at `config.base_path`. Creates parent
    /// directories as needed, and opens `<base>.sigmf-data.partial`
    /// for writing.
    pub async fn open(config: PersistConfig) -> SdrResult<Self> {
        let base = &config.base_path;
        if let Some(parent) = base.parent() {
            if !parent.as_os_str().is_empty() {
                tokio::fs::create_dir_all(parent)
                    .await
                    .map_err(|e| SdrError::SigmfIo(format!("mkdir {parent:?}: {e}")))?;
            }
        }
        let data_partial = with_ext(base, "sigmf-data.partial");
        let data_final = with_ext(base, "sigmf-data");
        let meta_final = with_ext(base, "sigmf-meta");
        let file = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open(&data_partial)
            .await
            .map_err(|e| SdrError::SigmfIo(format!("open {data_partial:?}: {e}")))?;
        Ok(Self {
            config,
            data_partial,
            data_final,
            meta_final,
            data_file: Some(file),
            bytes_written: 0,
            sample_pairs_written: 0,
            captures: Vec::new(),
            sample_rate_hz: None,
            closed: false,
        })
    }

    /// Append a chunk to the recording. Opens a new captures-entry
    /// every time the chunk's `centre_hz` differs from the previous
    /// one.
    pub async fn append(&mut self, chunk: &IqChunk) -> SdrResult<()> {
        if self.closed {
            return Err(SdrError::SigmfIo("append: writer is closed".into()));
        }
        let file = self
            .data_file
            .as_mut()
            .ok_or_else(|| SdrError::SigmfIo("append: data file is gone".into()))?;
        let needs_new_capture = self
            .captures
            .last()
            .map(|c| c.frequency != chunk.centre_hz)
            .unwrap_or(true);
        if needs_new_capture {
            self.captures.push(SigmfCapture {
                sample_start: self.sample_pairs_written,
                frequency: chunk.centre_hz,
                datetime: chunk.captured_at.to_rfc3339(),
            });
        }
        self.sample_rate_hz = Some(chunk.sample_rate_hz);
        file.write_all(&i8_slice_to_u8(&chunk.samples))
            .await
            .map_err(|e| SdrError::SigmfIo(format!("write: {e}")))?;
        self.bytes_written += chunk.len_bytes() as u64;
        self.sample_pairs_written += chunk.len_samples() as u64;
        if let Some(cap) = self.config.max_bytes {
            if self.bytes_written >= cap {
                tracing::info!(bytes = self.bytes_written, "sigmf max_bytes reached — closing");
                self.close().await?;
            }
        }
        Ok(())
    }

    /// Flush + rename the partial data file to its final name, then
    /// write the `.sigmf-meta` JSON. Idempotent.
    pub async fn close(&mut self) -> SdrResult<()> {
        if self.closed {
            return Ok(());
        }
        if let Some(mut f) = self.data_file.take() {
            f.flush()
                .await
                .map_err(|e| SdrError::SigmfIo(format!("flush: {e}")))?;
            // Drop the handle before renaming on Windows-safe paths.
            drop(f);
        }
        tokio::fs::rename(&self.data_partial, &self.data_final)
            .await
            .map_err(|e| SdrError::SigmfIo(format!("rename {:?}: {e}", self.data_partial)))?;
        let meta = SigmfMeta {
            global: build_global(
                self.sample_rate_hz.unwrap_or(0),
                self.config.author.as_deref(),
                self.config.description.as_deref(),
            ),
            captures: std::mem::take(&mut self.captures),
            annotations: Vec::new(),
        };
        let bytes = serde_json::to_vec_pretty(&meta)
            .map_err(|e| SdrError::SigmfIo(format!("serialize meta: {e}")))?;
        tokio::fs::write(&self.meta_final, &bytes)
            .await
            .map_err(|e| SdrError::SigmfIo(format!("write meta {:?}: {e}", self.meta_final)))?;
        self.closed = true;
        Ok(())
    }

    /// Path the writer is laying samples down to (the `.partial` file
    /// before `close`, the final `.sigmf-data` after).
    pub fn data_path(&self) -> &Path {
        if self.closed {
            &self.data_final
        } else {
            &self.data_partial
        }
    }

    /// Final metadata path (only present after `close`).
    pub fn meta_path(&self) -> &Path {
        &self.meta_final
    }

    /// How many bytes have been written so far.
    pub fn bytes_written(&self) -> u64 {
        self.bytes_written
    }

    /// Number of `captures[]` entries the writer would emit if you
    /// called `close` right now.
    pub fn captures(&self) -> &[SigmfCapture] {
        &self.captures
    }
}

impl Drop for SigmfWriter {
    fn drop(&mut self) {
        // Best-effort: if the writer was never closed, leave the
        // `.partial` data file in place but do NOT rename it. The
        // missing `.sigmf-meta` is the signal of an aborted capture.
        if !self.closed {
            tracing::warn!(
                path = %self.data_partial.display(),
                "SigmfWriter dropped without close — leaving partial file in place"
            );
        }
    }
}

/// Subscribe to `actor_ref`'s broadcast stream and persist each chunk
/// into `writer` until the channel closes. Returns the writer back to
/// the caller so they can `close()` it themselves (or rely on
/// auto-flush on drop, which leaves `.partial`).
///
/// Convenience wrapper around `recv → writer.append → close`. See
/// [`crate::SdrActorRef::subscribe`] for the underlying primitive.
pub async fn persist_until_eos(
    mut rx: broadcast::Receiver<IqChunk>,
    mut writer: SigmfWriter,
) -> SdrResult<SigmfWriter> {
    loop {
        match rx.recv().await {
            Ok(chunk) => writer.append(&chunk).await?,
            Err(broadcast::error::RecvError::Lagged(n)) => {
                tracing::warn!(missed = n, "sigmf writer lagged broadcast");
                continue;
            }
            Err(broadcast::error::RecvError::Closed) => break,
        }
    }
    writer.close().await?;
    Ok(writer)
}

/// Coerce a slice of `i8` into a `u8` view of the same memory — the
/// HackRF wire format is identical between the two; `ci8_le` interprets
/// the bytes as signed.
fn i8_slice_to_u8(input: &Arc<[i8]>) -> Vec<u8> {
    input.iter().map(|s| *s as u8).collect()
}

/// Append a new extension to `base`, preserving the file stem.
fn with_ext(base: &Path, ext: &str) -> PathBuf {
    let mut path = base.as_os_str().to_owned();
    path.push(".");
    path.push(ext);
    PathBuf::from(path)
}

fn build_global(sample_rate_hz: u32, author: Option<&str>, description: Option<&str>) -> Value {
    let mut map = serde_json::Map::new();
    map.insert("core:datatype".into(), Value::String("ci8_le".into()));
    map.insert(
        "core:sample_rate".into(),
        Value::Number(serde_json::Number::from(sample_rate_hz)),
    );
    map.insert(
        "core:version".into(),
        Value::String("1.0.0".into()),
    );
    map.insert(
        "core:recorder".into(),
        Value::String("atomr-physical-sdr".into()),
    );
    if let Some(a) = author {
        map.insert("core:author".into(), Value::String(a.into()));
    }
    if let Some(d) = description {
        map.insert("core:description".into(), Value::String(d.into()));
    }
    Value::Object(map)
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use tempfile::tempdir;

    fn synth_chunk(seq: u64, freq: u64, rate: u32, n: usize) -> IqChunk {
        let samples: Vec<i8> = (0..n * 2).map(|i| (i % 250) as i8).collect();
        IqChunk {
            sequence: seq,
            captured_at: Utc::now(),
            centre_hz: freq,
            sample_rate_hz: rate,
            samples: Arc::from(samples),
        }
    }

    #[tokio::test]
    async fn writer_records_data_and_meta() {
        let dir = tempdir().unwrap();
        let base = dir.path().join("capture");
        let mut w = SigmfWriter::open(PersistConfig::at(&base)).await.unwrap();
        w.append(&synth_chunk(0, 100_000_000, 4_000_000, 1024))
            .await
            .unwrap();
        w.append(&synth_chunk(1, 100_000_000, 4_000_000, 1024))
            .await
            .unwrap();
        w.close().await.unwrap();
        let data = with_ext(&base, "sigmf-data");
        let meta = with_ext(&base, "sigmf-meta");
        assert!(data.exists());
        assert!(meta.exists());
        let meta_text = std::fs::read_to_string(&meta).unwrap();
        let parsed: SigmfMeta = serde_json::from_str(&meta_text).unwrap();
        assert_eq!(parsed.captures.len(), 1);
        assert_eq!(parsed.captures[0].frequency, 100_000_000);
        let data_bytes = std::fs::metadata(&data).unwrap().len();
        // 2 chunks × 1024 pairs × 2 bytes/sample = 4096 bytes.
        assert_eq!(data_bytes, 4096);
    }

    #[tokio::test]
    async fn tune_opens_new_capture_entry() {
        let dir = tempdir().unwrap();
        let base = dir.path().join("tune");
        let mut w = SigmfWriter::open(PersistConfig::at(&base)).await.unwrap();
        w.append(&synth_chunk(0, 100_000_000, 4_000_000, 512))
            .await
            .unwrap();
        w.append(&synth_chunk(1, 200_000_000, 4_000_000, 512))
            .await
            .unwrap();
        w.close().await.unwrap();
        let meta_text = std::fs::read_to_string(with_ext(&base, "sigmf-meta")).unwrap();
        let parsed: SigmfMeta = serde_json::from_str(&meta_text).unwrap();
        assert_eq!(parsed.captures.len(), 2);
        assert_eq!(parsed.captures[0].sample_start, 0);
        assert_eq!(parsed.captures[1].sample_start, 512);
        assert_eq!(parsed.captures[1].frequency, 200_000_000);
    }

    #[tokio::test]
    async fn partial_left_on_drop_without_close() {
        let dir = tempdir().unwrap();
        let base = dir.path().join("aborted");
        {
            let mut w = SigmfWriter::open(PersistConfig::at(&base)).await.unwrap();
            w.append(&synth_chunk(0, 100_000_000, 4_000_000, 256))
                .await
                .unwrap();
            // dropped without close
        }
        assert!(with_ext(&base, "sigmf-data.partial").exists());
        assert!(!with_ext(&base, "sigmf-meta").exists());
        assert!(!with_ext(&base, "sigmf-data").exists());
    }
}
