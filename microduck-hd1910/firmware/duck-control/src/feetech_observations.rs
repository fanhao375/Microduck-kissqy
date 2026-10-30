//! Diagnostic replies are independent of the atomic sensor sample and motion gates.
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Instant;

use serde::{Serialize, Serializer};

use crate::NUM_JOINTS;
use crate::feetech::{IMU_ID, SYNC_ADDRESS, SYNC_LENGTH, ServoSample, Status};
use crate::feetech_snapshot::Metadata;

#[derive(Default)]
pub(super) struct Observations(BTreeMap<u8, Record>);

struct Record {
    checked_at: Instant,
    responding: bool,
    reply_at: Option<Instant>,
    feedback: Option<(Instant, ServoSample)>,
}

#[derive(Debug, Clone, Copy, Default, Serialize)]
struct Row {
    responding: Option<bool>,
    check_age_ms: Option<u64>,
    reply_age_ms: Option<u64>,
    feedback_age_ms: Option<u64>,
    position_raw: Option<i32>,
    speed_raw: Option<i32>,
    load_raw: Option<i32>,
    current_raw: Option<i32>,
    volts: Option<f64>,
    temperature_c: Option<u8>,
}

/// Fixed numeric storage; configured names remain in the existing shared metadata.
#[derive(Debug, Clone)]
pub struct Snapshot {
    captured_at: Instant,
    metadata: Arc<Metadata>,
    rows: [Row; NUM_JOINTS],
    attempted: bool,
}

impl Snapshot {
    pub fn empty(metadata: Arc<Metadata>) -> Self {
        Observations::default().snapshot(metadata, Instant::now())
    }
}

impl Default for Snapshot {
    fn default() -> Self {
        Self::empty(Arc::new(Metadata::default()))
    }
}

impl Observations {
    pub(super) fn requested(&mut self, ids: &[u8], now: Instant) {
        for id in ids.iter().copied().filter(|id| *id < 254 && *id != IMU_ID) {
            let record = self.0.entry(id).or_insert(Record {
                checked_at: now,
                responding: false,
                reply_at: None,
                feedback: None,
            });
            record.checked_at = now;
            record.responding = false;
        }
    }

    pub(super) fn replied(&mut self, status: &Status, address: u8, length: u8, now: Instant) {
        let Some(record) = self.0.get_mut(&status.id) else {
            return;
        };
        record.responding = true;
        record.reply_at = Some(now);
        if status.error == 0
            && address == SYNC_ADDRESS
            && length == SYNC_LENGTH
            && let Ok(sample) = ServoSample::decode(&status.data)
        {
            record.feedback = Some((now, sample));
        }
    }

    pub(super) fn snapshot(&self, metadata: Arc<Metadata>, now: Instant) -> Snapshot {
        let age = |at| now.saturating_duration_since(at).as_millis() as u64;
        let mut rows = [Row::default(); NUM_JOINTS];
        let mut attempted = false;
        for (row, joint) in rows.iter_mut().zip(&metadata.joints) {
            if let Some(record) = self.0.get(&joint.id) {
                attempted = true;
                row.responding = Some(record.responding);
                row.check_age_ms = Some(age(record.checked_at));
                row.reply_age_ms = record.reply_at.map(age);
                if let Some((at, sample)) = &record.feedback {
                    row.feedback_age_ms = Some(age(*at));
                    row.position_raw = Some(sample.feedback.position_raw);
                    row.speed_raw = Some(sample.feedback.speed_raw);
                    row.load_raw = Some(sample.feedback.load_raw);
                    row.current_raw = Some(sample.current_raw);
                    row.volts = Some(sample.feedback.volts);
                    row.temperature_c = Some(sample.feedback.temperature_c);
                }
            }
        }
        Snapshot {
            captured_at: now,
            metadata,
            rows,
            attempted,
        }
    }
}

impl Serialize for Snapshot {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::{SerializeSeq, SerializeStruct};
        struct Rows<'a>(&'a Snapshot);
        impl Serialize for Rows<'_> {
            fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                #[derive(Serialize)]
                struct NamedRow<'a> {
                    id: u8,
                    name: &'a str,
                    #[serde(flatten)]
                    row: &'a Row,
                }
                let mut sequence = serializer.serialize_seq(Some(self.0.metadata.joints.len()))?;
                for (row, joint) in self.0.rows.iter().zip(&self.0.metadata.joints) {
                    sequence.serialize_element(&NamedRow {
                        id: joint.id,
                        name: &joint.name,
                        row,
                    })?;
                }
                sequence.end()
            }
        }
        let mut object = serializer.serialize_struct("ServoObservations", 3)?;
        object.serialize_field("attempted", &self.attempted)?;
        object.serialize_field("age_ms", &(self.captured_at.elapsed().as_millis() as u64))?;
        object.serialize_field("rows", &Rows(self))?;
        object.end()
    }
}
