// Added 2026-10 by fanhao375 (microduck-replica): BNO08x gyro-integrated attitude; see IMU-MODULE.md at the repository root.
//! A BNO08x's reports as they arrived, one CSV line each — what the gyro fusion's numbers are to
//! be settled on, at the chip's 100 Hz rather than a recording's 50.
//!
//! Off unless `[trunk_imu] bno08x_raw_log` names a file. The reader thread never touches the
//! file: it formats one step's lines into a string and offers it to a bounded queue, and a
//! thread of the log's own writes them. A full queue drops the step and counts its lines —
//! the reader must never wait on a disk. The file is appended to; each run begins with a `#`
//! line naming the wall-clock time its `host_us` counts from, then the column names, and the
//! file stops growing at [`MAX_BYTES`].
//!
//! `host_us,packet,channel,shtp_seq,report_id,report_seq,status,delay_100us,v0,v1,v2,v3,v4,v5`
//!
//! - `host_us`: the reader step's time, µs since the log opened. Every report one step reads
//!   shares it.
//! - `packet`: packets read since the log opened; `channel`, `shtp_seq`: the SHTP header's.
//! - `report_id`, `report_seq`, `status`, `delay_100us`: an SH-2 input report's header —
//!   status bits 0–1 are the accuracy, and the delay is the 14-bit one after the batch's base
//!   timestamp, in 100 µs.
//! - `v0…`: the report's `i16` words as sent (gyro x y z in Q9, game rotation x y z w in Q14),
//!   a timestamp report's 32-bit value, a feature response's report and interval in µs. Blank
//!   where a report has fewer. A report that could not be parsed is its ID and `unparsed`.

use std::fmt::Write as _;
use std::io::Write as _;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use duck_control::bno08x::{self, ControlReply};
use duck_control::trunk_imu::RawLogHealth;

/// The file stops growing here: about an hour of reports.
pub(crate) const MAX_BYTES: u64 = 64 * 1024 * 1024;
/// Reader steps queued for the writer. A step is a few lines; this is seconds of slack.
const QUEUE: usize = 256;
const COLUMNS: &str =
    "host_us,packet,channel,shtp_seq,report_id,report_seq,status,delay_100us,v0,v1,v2,v3,v4,v5\n";

#[derive(Default)]
struct Stats {
    lines: AtomicU64,
    dropped: AtomicU64,
    bytes: AtomicU64,
    error: Mutex<Option<String>>,
}

impl Stats {
    fn fail(&self, why: String) {
        tracing::warn!(reason = %why, "BNO08x raw report log is not being written");
        if let Ok(mut error) = self.error.lock() {
            *error = Some(why);
        }
    }
}

struct Inner {
    path: String,
    tx: Option<SyncSender<String>>,
    origin: Instant,
    packets: AtomicU64,
    stats: Arc<Stats>,
}

/// One log, shared by every chip state machine the reader makes over its life.
#[derive(Clone)]
pub(crate) struct RawLog {
    inner: Arc<Inner>,
}

impl RawLog {
    /// Open `path` for appending and start its writer. Never fails: a file that cannot be
    /// written is named in [`Self::health`] and the log writes nothing — a diagnostic must not
    /// keep the robot's IMU from starting.
    pub(crate) fn open(path: &str) -> Self {
        Self::open_with(path, QUEUE)
    }

    fn open_with(path: &str, queue: usize) -> Self {
        let stats = Arc::new(Stats::default());
        let origin = Instant::now();
        let tx = match Self::start(path, queue, &stats) {
            Ok(tx) => Some(tx),
            Err(why) => {
                stats.fail(why);
                None
            }
        };
        Self {
            inner: Arc::new(Inner {
                path: path.to_owned(),
                tx,
                origin,
                packets: AtomicU64::new(0),
                stats,
            }),
        }
    }

    fn start(path: &str, queue: usize, stats: &Arc<Stats>) -> Result<SyncSender<String>, String> {
        let mut file = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .map_err(|e| format!("cannot open {path}: {e}"))?;
        let size = file.metadata().map(|m| m.len()).unwrap_or(0);
        if size >= MAX_BYTES {
            return Err(format!(
                "{path} already holds {size} bytes, the cap is {MAX_BYTES}; move it away to log again"
            ));
        }
        let wall = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        let head = format!(
            "# robotd {}: BNO08x reports; host_us counts from unix time {wall:.6}\n{COLUMNS}",
            env!("CARGO_PKG_VERSION")
        );
        file.write_all(head.as_bytes())
            .and_then(|()| file.flush())
            .map_err(|e| format!("cannot write {path}: {e}"))?;
        stats.bytes.store(head.len() as u64, Ordering::Relaxed);
        let (tx, rx) = std::sync::mpsc::sync_channel(queue);
        let writer_stats = Arc::clone(stats);
        let size = size + head.len() as u64;
        let label = path.to_owned();
        std::thread::Builder::new()
            .name("imu-raw-log".into())
            .spawn(move || write(rx, file, size, &label, &writer_stats))
            .map_err(|e| format!("cannot start the writer for {path}: {e}"))?;
        Ok(tx)
    }

    /// A buffer for one reader step at `now`, offered to the writer when it is dropped.
    pub(crate) fn step(&self, now: Instant) -> StepLines<'_> {
        StepLines {
            log: self,
            host_us: now.saturating_duration_since(self.inner.origin).as_micros() as u64,
            text: String::new(),
            lines: 0,
        }
    }

    pub(crate) fn health(&self) -> RawLogHealth {
        let stats = &self.inner.stats;
        RawLogHealth {
            path: self.inner.path.clone(),
            lines: stats.lines.load(Ordering::Relaxed),
            dropped: stats.dropped.load(Ordering::Relaxed),
            bytes: stats.bytes.load(Ordering::Relaxed),
            error: stats.error.lock().ok().and_then(|e| e.clone()),
        }
    }
}

/// Reader steps written between two flushes at most, so a writer that is falling behind
/// still puts what it has on disk.
const STEPS_PER_FLUSH: usize = 64;

/// The writer thread: the steps queued, flushed whenever the queue runs dry (or every
/// [`STEPS_PER_FLUSH`]), counted once they are on disk, until the reader is gone, the file is
/// full or a write fails.
fn write(rx: Receiver<String>, file: std::fs::File, mut size: u64, path: &str, stats: &Stats) {
    let mut out = std::io::BufWriter::new(file);
    while let Ok(first) = rx.recv() {
        let (mut pending, mut batch) = (0u64, Some(first));
        for _ in 0..STEPS_PER_FLUSH {
            let Some(text) = batch.take().or_else(|| rx.try_recv().ok()) else {
                break;
            };
            let lines = text.matches('\n').count() as u64;
            if size + text.len() as u64 > MAX_BYTES {
                stats.dropped.fetch_add(lines, Ordering::Relaxed);
                if stats.error.lock().map(|e| e.is_none()).unwrap_or(false) {
                    stats.fail(format!("{path} reached its cap of {MAX_BYTES} bytes"));
                }
                continue;
            }
            if let Err(e) = out.write_all(text.as_bytes()) {
                stats.dropped.fetch_add(lines + pending, Ordering::Relaxed);
                stats.fail(format!("cannot write {path}: {e}"));
                return;
            }
            size += text.len() as u64;
            pending += lines;
        }
        if let Err(e) = out.flush() {
            stats.dropped.fetch_add(pending, Ordering::Relaxed);
            stats.fail(format!("cannot write {path}: {e}"));
            return;
        }
        stats.lines.fetch_add(pending, Ordering::Relaxed);
        stats.bytes.store(size, Ordering::Relaxed);
    }
}

/// One reader step's lines.
pub(crate) struct StepLines<'a> {
    log: &'a RawLog,
    host_us: u64,
    text: String,
    lines: u64,
}

impl StepLines<'_> {
    /// Every report in one packet, a line each.
    pub(crate) fn packet(&mut self, channel: u8, shtp_seq: u8, payload: &[u8]) {
        let packet = self.log.inner.packets.fetch_add(1, Ordering::Relaxed) + 1;
        let lead = format!("{},{packet},{channel},{shtp_seq}", self.host_us);
        if channel == bno08x::channel::CONTROL {
            match bno08x::control_replies(payload) {
                Ok(replies) => {
                    for reply in replies {
                        let fields = match reply {
                            ControlReply::Feature {
                                report_id,
                                interval_us,
                            } => format!(
                                "{},,,,{report_id},{interval_us}",
                                bno08x::GET_FEATURE_RESPONSE
                            ),
                            ControlReply::ProductId(id) => format!(
                                "{},,,,{},{},{},{},{}",
                                bno08x::PRODUCT_ID_RESPONSE,
                                id.part_number,
                                id.version.0,
                                id.version.1,
                                id.version.2,
                                id.build
                            ),
                            ControlReply::Other(id) => format!("{id},,,"),
                        };
                        self.line(&lead, &fields);
                    }
                }
                Err(_) => self.unparsed(&lead, payload.first().copied()),
            }
            return;
        }
        if !matches!(
            channel,
            bno08x::channel::REPORTS | bno08x::channel::WAKE_REPORTS
        ) {
            self.unparsed(&lead, payload.first().copied());
            return;
        }
        for report in bno08x::reports(payload) {
            match report {
                Ok(report) => {
                    let fields = Self::report(report);
                    self.line(&lead, &fields);
                }
                Err(e) => {
                    // The parse ends at the first report it cannot place; name it.
                    let id = match e {
                        bno08x::Bno08xError::UnknownReport(id)
                        | bno08x::Bno08xError::TruncatedReport(id) => Some(id),
                        _ => None,
                    };
                    self.unparsed(&lead, id);
                }
            }
        }
    }

    fn report(report: &[u8]) -> String {
        let id = report[0];
        if matches!(id, 0xFA | 0xFB) {
            let value = u32::from_le_bytes([report[1], report[2], report[3], report[4]]);
            return if id == 0xFA {
                format!("{id},,,,{}", value as i32)
            } else {
                format!("{id},,,,{value}")
            };
        }
        let (sequence, status) = (report[1], report[2]);
        let delay = (u16::from(status >> 2) << 8) | u16::from(report[3]);
        let mut fields = format!("{id},{sequence},{status},{delay}");
        for word in report[4..].as_chunks::<2>().0.iter().take(6) {
            let _ = write!(fields, ",{}", i16::from_le_bytes(*word));
        }
        fields
    }

    fn unparsed(&mut self, lead: &str, id: Option<u8>) {
        let id = id.map(|id| id.to_string()).unwrap_or_default();
        self.line(lead, &format!("{id},,,,unparsed"));
    }

    fn line(&mut self, lead: &str, fields: &str) {
        self.text.push_str(lead);
        self.text.push(',');
        self.text.push_str(fields);
        self.text.push('\n');
        self.lines += 1;
    }
}

impl Drop for StepLines<'_> {
    fn drop(&mut self) {
        if self.lines == 0 {
            return;
        }
        let stats = &self.log.inner.stats;
        let Some(tx) = &self.log.inner.tx else {
            stats.dropped.fetch_add(self.lines, Ordering::Relaxed);
            return;
        };
        match tx.try_send(std::mem::take(&mut self.text)) {
            Ok(()) => {}
            Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) => {
                stats.dropped.fetch_add(self.lines, Ordering::Relaxed);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn report(id: u8, sequence: u8, status: u8, delay: u8, values: &[i16]) -> Vec<u8> {
        let mut out = vec![id, sequence, status, delay];
        for v in values {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }

    fn wait_for(log: &RawLog, lines: u64) -> RawLogHealth {
        for _ in 0..200 {
            let health = log.health();
            if health.lines + health.dropped >= lines {
                return health;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        log.health()
    }

    /// Every report a line, in order, with the SH-2 header split out and the words raw; a
    /// feature response on the control channel says which report and what period.
    #[test]
    fn reports_are_logged_a_line_each_with_their_raw_words() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("raw.csv");
        let log = RawLog::open(path.to_str().unwrap());
        let now = log.inner.origin + Duration::from_micros(1500);
        assert_eq!(log.health().error, None);
        {
            let mut step = log.step(now);
            let mut payload = vec![0xFB, 0x10, 0, 0, 0];
            payload.extend(report(
                bno08x::GYROSCOPE,
                7,
                0b0000_0111,
                9,
                &[512, -512, 3],
            ));
            payload.extend(report(
                bno08x::GAME_ROTATION_VECTOR,
                7,
                3,
                0,
                &[0, 0, 0, 16384],
            ));
            step.packet(bno08x::channel::REPORTS, 41, &payload);
            let mut feature = [0u8; 17];
            feature[0] = bno08x::GET_FEATURE_RESPONSE;
            feature[1] = bno08x::GAME_ROTATION_VECTOR;
            feature[5..9].copy_from_slice(&10_000u32.to_le_bytes());
            step.packet(bno08x::channel::CONTROL, 2, &feature);
        }
        let health = wait_for(&log, 4);
        assert_eq!((health.lines, health.dropped, health.error), (4, 0, None));
        let text = std::fs::read_to_string(&path).unwrap();
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[0].starts_with("# robotd "), "{}", lines[0]);
        assert_eq!(lines[1], COLUMNS.trim_end());
        assert_eq!(lines[2], "1500,1,3,41,251,,,,16");
        assert_eq!(
            lines[3], "1500,1,3,41,2,7,7,265,512,-512,3",
            "delay = (7 >> 2) << 8 | 9"
        );
        assert_eq!(lines[4], "1500,1,3,41,8,7,3,0,0,0,0,16384");
        assert_eq!(lines[5], "1500,2,2,2,252,,,,8,10000");
        assert_eq!(health.bytes, text.len() as u64);
    }

    /// A report the parse cannot place is named, not guessed at.
    #[test]
    fn an_unparsable_report_is_named() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("raw.csv");
        let log = RawLog::open(path.to_str().unwrap());
        log.step(log.inner.origin)
            .packet(bno08x::channel::REPORTS, 0, &[0x42, 0, 0, 0]);
        wait_for(&log, 1);
        let text = std::fs::read_to_string(&path).unwrap();
        assert_eq!(text.lines().last(), Some("0,1,3,0,66,,,,unparsed"));
    }

    /// The reader never waits: with the writer's queue full, a step is dropped and its lines
    /// counted. Here the queue holds nothing and the writer is never given a turn.
    #[test]
    fn a_full_queue_drops_and_counts_rather_than_waiting() {
        let stats = Arc::new(Stats::default());
        let (tx, _rx) = std::sync::mpsc::sync_channel(0);
        let log = RawLog {
            inner: Arc::new(Inner {
                path: "test".into(),
                tx: Some(tx),
                origin: Instant::now(),
                packets: AtomicU64::new(0),
                stats,
            }),
        };
        let started = Instant::now();
        for _ in 0..100 {
            let mut payload = report(bno08x::GYROSCOPE, 1, 3, 0, &[0, 0, 0]);
            payload.extend(report(
                bno08x::GAME_ROTATION_VECTOR,
                1,
                3,
                0,
                &[0, 0, 0, 16384],
            ));
            log.step(Instant::now())
                .packet(bno08x::channel::REPORTS, 0, &payload);
        }
        assert!(started.elapsed() < Duration::from_secs(1));
        let health = log.health();
        assert_eq!((health.lines, health.dropped), (0, 200));
    }

    /// A file that cannot be opened is named in the health, and the log simply writes nothing.
    #[test]
    fn a_path_that_cannot_be_written_is_reported_not_fatal() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("no-such-dir").join("raw.csv");
        let log = RawLog::open(path.to_str().unwrap());
        log.step(Instant::now())
            .packet(bno08x::channel::REPORTS, 0, &[0xFB, 0, 0, 0, 0]);
        let health = log.health();
        assert!(
            health
                .error
                .as_deref()
                .is_some_and(|e| e.contains("cannot open")),
            "{health:?}"
        );
        assert_eq!((health.lines, health.dropped), (0, 1));
    }

    /// Each run appends, under a `#` line of its own.
    #[test]
    fn a_second_run_appends_under_its_own_header() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("raw.csv");
        for _ in 0..2 {
            let log = RawLog::open(path.to_str().unwrap());
            log.step(log.inner.origin)
                .packet(bno08x::channel::REPORTS, 0, &[0xFB, 1, 0, 0, 0]);
            wait_for(&log, 1);
        }
        let text = std::fs::read_to_string(&path).unwrap();
        assert_eq!(text.matches("# robotd ").count(), 2);
        assert_eq!(text.matches("0,1,3,0,251,,,,1\n").count(), 2);
    }
}
