// Added 2026-10 by fanhao375 (microduck-replica): UDP pad ([netpad]) and pad-loss sit; after Pollen Robotics microduck 0.17.1 netpadd (Apache-2.0); see IMU-MODULE.md at the repository root.
//! The pad over UDP: `[netpad] enabled`, driven by the official `duckctl udp-pad` (Pollen
//! microduck 0.17), or by anything that speaks its 28-byte datagram ([`wire`]).
//!
//! Upstream reads it in a daemon of its own, `netpadd`, through a mapping it shares with `padd`.
//! This `padd` has its own mapping — the R17 pad, with its gates, roller holds and web-pad
//! priorities — so here UDP is a second **source** for that one mapping instead: a datagram's
//! buttons become the same gilrs `Button` edges a Bluetooth pad's events are, its sticks the same
//! axes. Whatever the Bluetooth pad does on this robot, the UDP pad does, by the same code.
//!
//! This module is the I/O around it: one socket, an answer to every datagram ([`wire::Status`])
//! so the sender can see the link, and once a second the robot's health to the client that drives
//! ([`wire::RobotReport`]), kept fresh off the pad loop by [`health`].

pub mod health;
pub mod receiver;
pub mod wire;

use std::net::{SocketAddr, UdpSocket};
use std::path::Path;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use gilrs::{Axis, Button};

use receiver::{Accept, Edge, Frame, Receiver};
use wire::{Buttons, RobotReport, Status, Verdict};

/// The wire's buttons and the gilrs ones the mapping is written in. Bit 12, Home, is gilrs's
/// `Mode`, which this pad mapping does not use. The wire has no stick clicks, so the R17 L3
/// roller test is a Bluetooth-pad-only control.
const BUTTONS: [(Buttons, Button); 13] = [
    (Buttons::A, Button::South),
    (Buttons::B, Button::East),
    (Buttons::X, Button::West),
    (Buttons::Y, Button::North),
    // gilrs names the bumpers `LeftTrigger`/`RightTrigger`; the analog triggers are `…2`.
    (Buttons::LB, Button::LeftTrigger),
    (Buttons::RB, Button::RightTrigger),
    (Buttons::START, Button::Start),
    (Buttons::SELECT, Button::Select),
    (Buttons::UP, Button::DPadUp),
    (Buttons::DOWN, Button::DPadDown),
    (Buttons::LEFT, Button::DPadLeft),
    (Buttons::RIGHT, Button::DPadRight),
    (Buttons::HOME, Button::Mode),
];

fn wire_bit(button: Button) -> Option<Buttons> {
    BUTTONS.iter().find(|(_, b)| *b == button).map(|(w, _)| *w)
}

fn gilrs_button(bit: Buttons) -> Option<Button> {
    BUTTONS.iter().find(|(w, _)| *w == bit).map(|(_, b)| *b)
}

/// A button edge in the mapping's vocabulary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PadEdge {
    Pressed(Button),
    Released(Button),
}

impl Frame {
    /// The edges since the last tick, as gilrs buttons, in the order they happened.
    pub fn pad_edges(&self) -> impl Iterator<Item = PadEdge> + '_ {
        self.edges.iter().filter_map(|edge| match *edge {
            Edge::Pressed(b) => gilrs_button(b).map(PadEdge::Pressed),
            Edge::Released(b) => gilrs_button(b).map(PadEdge::Released),
        })
    }

    pub fn is_pressed(&self, button: Button) -> bool {
        wire_bit(button).is_some_and(|b| self.held.contains(b))
    }

    /// Whether the button was down where this frame's edges start — for a session's first frame,
    /// in the datagram that started it. The baseline for gating presses the robot never saw.
    pub fn was_pressed(&self, button: Button) -> bool {
        wire_bit(button).is_some_and(|b| self.before.contains(b))
    }

    /// A stick, -1..1 with up positive — what gilrs gives for the same stick.
    pub fn value(&self, axis: Axis) -> f32 {
        let i = match axis {
            Axis::LeftStickX => 0,
            Axis::LeftStickY => 1,
            Axis::RightStickX => 2,
            Axis::RightStickY => 3,
            _ => return 0.0,
        };
        self.values[i] as f32
    }

    /// An analog trigger, 0..1 — what gilrs's `button_data(…Trigger2).value()` gives.
    pub fn trigger(&self, button: Button) -> f64 {
        match button {
            Button::LeftTrigger2 => self.values[4],
            Button::RightTrigger2 => self.values[5],
            _ => 0.0,
        }
    }

    pub fn name(&self) -> String {
        format!("UDP pad {}", self.peer)
    }
}

/// What the pad loop should do this tick.
#[derive(Debug)]
pub enum Step {
    /// A client is driving; this is the pad now.
    Driving(Frame),
    /// The client that was driving went quiet past the timeout, just now.
    Gone,
    /// Nobody is driving.
    Absent,
}

/// The last request the pad asked for and the robot refused, for the report. Written by
/// `request()` in `main.rs`, whichever source is driving; only the UDP pad has anywhere to show it.
static REFUSED: Mutex<Option<(&'static str, String, Instant)>> = Mutex::new(None);

pub fn note_refusal(method: &'static str, reason: String) {
    if let Ok(mut r) = REFUSED.lock() {
        *r = Some((method, reason, Instant::now()));
    }
}

/// Rate limit for the two log lines a stranger on the LAN could otherwise make fill the journal.
const NOISE_LOG_EVERY: Duration = Duration::from_secs(10);

/// The longest an answer may wait to go out before it is dropped.
const SEND_TIMEOUT: Duration = Duration::from_millis(5);

/// At most this many queued datagrams are read in one step, so a flood cannot hold the tick.
const DRAIN_MAX: usize = 256;

/// How a datagram was read, which decides what it may say about the client.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Read {
    /// As it landed, in a wait: its time is its arrival, so silence before it is judged, and it
    /// proves the client is there now.
    Live,
    /// Queued for at most the length of one pass of the loop: it keeps the session, nothing more.
    Queued,
    /// Queued while the loop was held up past `timeout`, for an unknown time: it keeps (or
    /// starts) the session, but its sticks are not trusted — `Receiver::unverified`.
    Stale,
}

pub struct Link {
    socket: UdpSocket,
    rx: Receiver,
    timeout: Duration,
    /// When the socket was last read through to empty — the end of a wait, or a drain that found
    /// nothing more. Anything queued behind it is at most this old.
    polled: Instant,
    /// A hold-up's queue is not read out yet: what is read next is `Read::Stale`.
    backlog: bool,
    /// `Read::Stale` datagrams accepted since the last step, for one log line.
    stale_reads: u32,
    #[cfg(test)]
    stalls: u32,
    answers: Answers,
    malformed: u32,
    /// The client expired inside a wait; the next step says so before anything else.
    gone: bool,
    report_at: Instant,
    health: health::Health,
    reserved_logged: bool,
    noise_logged: Option<Instant>,
}

impl Link {
    /// Bind the port and start keeping the robot's health for the report.
    pub fn bind(port: u16, timeout: Duration, robot: &Path) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", port))?;
        // An answer that cannot go out at once is dropped rather than waited for: the sender
        // reads the next one, and this loop drives a robot. Upstream sends with `try_send_to`.
        socket.set_write_timeout(Some(SEND_TIMEOUT))?;
        Ok(Self {
            socket,
            rx: Receiver::new(timeout),
            timeout,
            polled: Instant::now(),
            backlog: false,
            stale_reads: 0,
            #[cfg(test)]
            stalls: 0,
            answers: Answers::default(),
            malformed: 0,
            gone: false,
            report_at: Instant::now(),
            health: health::spawn(robot),
            reserved_logged: false,
            noise_logged: None,
        })
    }

    #[cfg(test)]
    pub fn local_addr(&self) -> Option<SocketAddr> {
        self.socket.local_addr().ok()
    }

    /// Read and answer datagrams until `until`. With `early`, return as soon as a client is
    /// waiting to be taken — the idle wait, so the first datagram after a quiet spell is not left
    /// for half a second.
    pub fn wait(&mut self, until: Instant, early: bool) {
        self.check_hold_up();
        self.wait_inner(until, early);
        if !self.backlog {
            self.polled = Instant::now();
        }
    }

    /// The loop has been away from the socket for `timeout` or more — a slow answer from robotd —
    /// so what queued meanwhile is of unknown age until the socket is read empty.
    fn check_hold_up(&mut self) {
        if Instant::now().duration_since(self.polled) >= self.timeout {
            self.backlog = true;
        }
    }

    fn wait_inner(&mut self, until: Instant, early: bool) {
        let mut buf = [0u8; 128];
        loop {
            if early && self.rx.session_pending() {
                return;
            }
            let now = Instant::now();
            let Some(left) = until.checked_duration_since(now).filter(|d| !d.is_zero()) else {
                return;
            };
            if self.socket.set_read_timeout(Some(left)).is_err() {
                std::thread::sleep(left);
                return;
            }
            match self.socket.recv_from(&mut buf) {
                // Read as it lands, so the time it is read is the time it arrived: a client quiet
                // past the timeout is gone *before* this datagram, which then starts a new session
                // — exactly as upstream, which wakes at the deadline.
                Ok((n, from)) => {
                    let read = if self.backlog {
                        Read::Stale
                    } else {
                        Read::Live
                    };
                    self.handle(Instant::now(), from, &buf[..n], read);
                }
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    // Waited and nothing came: the queue is empty, and what lands next is live.
                    self.backlog = false;
                    return;
                }
                Err(e) => {
                    // An ICMP error from a reply to a client that left can surface here on some
                    // stacks; it is about that client, not this socket. Bounded, so an error that
                    // repeats cannot spin the core this loop shares with the robot.
                    tracing::debug!(error = %e, "udp pad: recv");
                    std::thread::sleep(left.min(Duration::from_millis(5)));
                }
            }
        }
    }

    /// Everything already queued on the socket, without waiting and without judging silence: it
    /// was queued, so the client was talking while this loop was busy elsewhere — a slow answer
    /// from robotd must not read as the pad going away. After a hold-up it is `Read::Stale`.
    fn drain(&mut self) {
        self.check_hold_up();
        if self.socket.set_nonblocking(true).is_err() {
            return;
        }
        let mut buf = [0u8; 128];
        let mut emptied = false;
        for _ in 0..DRAIN_MAX {
            match self.socket.recv_from(&mut buf) {
                Ok((n, from)) => {
                    let read = if self.backlog {
                        Read::Stale
                    } else {
                        Read::Queued
                    };
                    self.handle(Instant::now(), from, &buf[..n], read);
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    emptied = true;
                    break;
                }
                Err(e) => tracing::debug!(error = %e, "udp pad: recv"),
            }
        }
        if let Err(e) = self.socket.set_nonblocking(false) {
            // Left non-blocking, every wait would return at once and this loop would spin.
            tracing::error!(error = %e, "udp pad: cannot make the socket blocking again");
        }
        // Read to the end: whatever lands from here is no older than this. Not when the cap cut
        // the read short — the rest of a hold-up's queue is still behind it.
        if emptied {
            self.backlog = false;
            self.polled = Instant::now();
        }
    }

    fn handle(&mut self, now: Instant, from: SocketAddr, bytes: &[u8], read: Read) {
        if read == Read::Live && self.rx.expire(now) {
            self.gone = true;
        }
        let accept = self.rx.on_datagram(now, from, bytes);
        if matches!(accept, Accept::Connected | Accept::Taken) {
            match read {
                Read::Live => self.rx.live(),
                Read::Queued => {}
                Read::Stale => {
                    self.rx.unverified();
                    self.stale_reads += 1;
                }
            }
        }
        self.log(accept, from, now);
        if let Some(status) = self.answers.on(accept, bytes, self.malformed, now) {
            self.send(&status.encode(), from);
        }
    }

    pub fn step(&mut self, now: Instant) -> Step {
        self.drain();
        if self.stale_reads > 0 {
            // Said once per hold-up, when there was something it changed: a normal disconnect is
            // not a hold-up, and a hold-up with nobody sending changes nothing.
            tracing::warn!(
                datagrams = self.stale_reads,
                "udp pad: the pad loop was held up past timeout_ms (a slow robotd?) — what queued \
                 meanwhile keeps the pad connected, but its sticks read as centred until a \
                 datagram is heard live"
            );
            self.stale_reads = 0;
            #[cfg(test)]
            {
                self.stalls += 1;
            }
        }
        if self.rx.expire(now) {
            self.gone = true;
        }
        if std::mem::take(&mut self.gone) {
            return Step::Gone;
        }
        match self.rx.take() {
            Some(frame) => {
                if frame.new_session {
                    // A client that just took the robot hears what it is now, not in a second.
                    self.report_at = now;
                }
                Step::Driving(frame)
            }
            None => Step::Absent,
        }
    }

    /// Once a second, the robot's health to the client that drives.
    pub fn report(&mut self, now: Instant) {
        let Some(peer) = self.rx.peer() else { return };
        if now < self.report_at {
            return;
        }
        self.report_at = now + wire::REPORT_EVERY;
        let snapshot = self.health.snapshot();
        let refused = REFUSED.lock().ok().and_then(|r| r.clone());
        let report = robot_report(&snapshot, refused.as_ref(), now);
        self.send(&report.encode(), peer);
    }

    fn send(&self, bytes: &[u8], to: SocketAddr) {
        if let Err(e) = self.socket.send_to(bytes, to) {
            tracing::debug!(error = %e, %to, "udp pad: answer not sent");
        }
    }

    fn log(&mut self, accept: Accept, from: SocketAddr, now: Instant) {
        match accept {
            Accept::Malformed(e) => {
                self.malformed = self.malformed.wrapping_add(1);
                if self.noise_due(now) {
                    tracing::info!(%from, error = %e, "udp pad: not a pad datagram — ignored");
                }
            }
            Accept::OtherPeer if self.noise_due(now) => {
                tracing::warn!(
                    %from, driving = ?self.rx.peer(),
                    "udp pad: another client is sending while one drives — ignored until it has been quiet"
                );
            }
            _ => {}
        }
        let reserved = self.rx.reserved_bits_seen();
        if reserved != 0 && !self.reserved_logged {
            self.reserved_logged = true;
            tracing::info!(
                bits = format_args!("{reserved:#06x}"),
                "udp pad: button bits this build does not know — ignored"
            );
        }
    }

    fn noise_due(&mut self, now: Instant) -> bool {
        if self
            .noise_logged
            .is_some_and(|at| now.duration_since(at) < NOISE_LOG_EVERY)
        {
            return false;
        }
        self.noise_logged = Some(now);
        true
    }
}

/// The answer to each datagram — upstream's `netpadd` `Answers`, unchanged. Nothing for a
/// malformed one: whatever sent it is not a pad client. A sender that is not the driving one hears
/// "busy" at most every `BUSY_EVERY`.
#[derive(Default)]
struct Answers {
    accepted: u32,
    stale: u32,
    busy_answered: Option<Instant>,
}

impl Answers {
    fn on(&mut self, accept: Accept, bytes: &[u8], malformed: u32, now: Instant) -> Option<Status> {
        let verdict = match accept {
            Accept::Connected | Accept::Taken => {
                self.accepted = self.accepted.wrapping_add(1);
                Verdict::Driving
            }
            Accept::Stale => {
                self.stale = self.stale.wrapping_add(1);
                Verdict::Driving
            }
            Accept::OtherPeer => {
                if self
                    .busy_answered
                    .is_some_and(|at| now.duration_since(at) < wire::BUSY_EVERY)
                {
                    return None;
                }
                self.busy_answered = Some(now);
                Verdict::Busy
            }
            Accept::Malformed(_) => return None,
        };
        let seq = wire::Packet::decode(bytes).ok()?.seq;
        Some(Status {
            verdict,
            seq,
            accepted: self.accepted,
            stale: self.stale,
            malformed,
        })
    }
}

fn robot_report(
    snapshot: &health::Snapshot,
    refused: Option<&(&'static str, String, Instant)>,
    now: Instant,
) -> RobotReport {
    RobotReport {
        health: snapshot.health.clone(),
        health_error: snapshot.health_error.clone(),
        posture: snapshot.posture.clone(),
        posture_error: snapshot.posture_error.clone(),
        refused: refused
            .filter(|(_, _, at)| now.duration_since(*at) < wire::REFUSAL_KEPT)
            .map(|(method, reason, at)| wire::Refusal {
                method: (*method).to_owned(),
                reason: reason.clone(),
                ago_ms: now.duration_since(*at).as_millis() as u64,
            }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wire::Packet;

    #[test]
    fn every_wire_button_has_one_gilrs_button_and_back() {
        for (bit, button) in BUTTONS {
            assert_eq!(gilrs_button(bit), Some(button));
            assert_eq!(wire_bit(button), Some(bit));
        }
        let all = BUTTONS.iter().fold(0u16, |acc, (b, _)| acc | b.0);
        assert_eq!(all, Buttons::KNOWN, "the 13 known bits, each once");
        assert_eq!(
            wire_bit(Button::LeftThumb),
            None,
            "no stick clicks on the wire"
        );
    }

    /// The frame reads like the gilrs pad it stands in for: X is `West`, the bumpers are
    /// `LeftTrigger`/`RightTrigger`, the analog triggers `…2`, and stick up is positive.
    #[test]
    fn a_frame_reads_like_a_gilrs_pad() {
        let mut rx = Receiver::new(Duration::from_millis(250));
        let t = Instant::now();
        let peer: SocketAddr = "10.0.0.2:5000".parse().unwrap();
        let held = Buttons::X | Buttons::LB;
        let p = Packet::from_axes(1, 0, [0.0, 1.0, -0.5, 0.0], [0.25, 1.0], held);
        rx.on_datagram(t, peer, &p.encode());
        let f = rx.take().unwrap();
        assert!(f.is_pressed(Button::West) && f.is_pressed(Button::LeftTrigger));
        assert!(!f.is_pressed(Button::South) && !f.is_pressed(Button::LeftTrigger2));
        assert_eq!(f.value(Axis::LeftStickY), 1.0);
        assert!((f.value(Axis::RightStickX) + 0.5).abs() < 1e-4);
        assert!((f.trigger(Button::LeftTrigger2) - 0.25).abs() < 1e-4);
        assert!((f.trigger(Button::RightTrigger2) - 1.0).abs() < 1e-4);
        assert_eq!(f.name(), "UDP pad 10.0.0.2:5000");

        let p = Packet::from_axes(2, 0, [0.0; 4], [0.0; 2], Buttons::START);
        rx.on_datagram(t, peer, &p.encode());
        let edges: Vec<_> = rx.take().unwrap().pad_edges().collect();
        assert_eq!(
            edges,
            [
                PadEdge::Released(Button::West),
                PadEdge::Released(Button::LeftTrigger),
                PadEdge::Pressed(Button::Start),
            ]
        );
    }

    #[test]
    fn the_driving_client_hears_driving_and_another_hears_busy_once_a_second() {
        let mut a = Answers::default();
        let t = Instant::now();
        let bytes = Packet {
            seq: 9,
            ..Default::default()
        }
        .encode();
        let s = a.on(Accept::Connected, &bytes, 0, t).unwrap();
        assert_eq!((s.verdict, s.seq, s.accepted), (Verdict::Driving, 9, 1));
        let s = a.on(Accept::Stale, &bytes, 3, t).unwrap();
        assert_eq!((s.stale, s.malformed), (1, 3));
        assert_eq!(
            a.on(Accept::OtherPeer, &bytes, 0, t).unwrap().verdict,
            Verdict::Busy
        );
        assert!(
            a.on(Accept::OtherPeer, &bytes, 0, t + Duration::from_millis(500))
                .is_none()
        );
        assert!(
            a.on(Accept::OtherPeer, &bytes, 0, t + Duration::from_secs(1))
                .is_some()
        );
        assert!(
            a.on(Accept::Malformed(wire::WireError::Magic), &bytes, 0, t)
                .is_none()
        );
    }

    #[test]
    fn a_refusal_is_reported_for_thirty_seconds() {
        let t = Instant::now();
        let refused = ("robot.do", "not home yet".to_owned(), t);
        let r = robot_report(
            &health::Snapshot::default(),
            Some(&refused),
            t + Duration::from_secs(2),
        );
        let r = r.refused.unwrap();
        assert_eq!((r.method.as_str(), r.ago_ms), ("robot.do", 2000));
        let r = robot_report(
            &health::Snapshot::default(),
            Some(&refused),
            t + Duration::from_secs(30),
        );
        assert!(r.refused.is_none());
    }

    /// Through a real socket on loopback: a datagram is answered with a status, the step that
    /// follows drives, and silence past the timeout is `Gone` once and then `Absent`.
    #[test]
    fn a_datagram_on_loopback_is_answered_and_drives_until_it_goes_quiet() {
        let dir = tempfile::tempdir().unwrap();
        let mut link =
            Link::bind(0, Duration::from_millis(150), &dir.path().join("none.sock")).unwrap();
        let port = link.local_addr().unwrap().port();
        let client = UdpSocket::bind("127.0.0.1:0").unwrap();
        client
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let p = Packet::from_axes(1, 0, [0.0, 0.5, 0.0, 0.0], [0.0; 2], Buttons::NONE);
        client.send_to(&p.encode(), ("127.0.0.1", port)).unwrap();

        link.wait(Instant::now() + Duration::from_secs(1), true);
        let mut answer = [0u8; 2048];
        let n = client.recv(&mut answer).unwrap();
        assert_eq!(&answer[..6], b"DKST\x01\x01", "driving");
        assert_eq!(n, wire::STATUS_LEN);
        match link.step(Instant::now()) {
            Step::Driving(f) => assert!((f.value(Axis::LeftStickY) - 0.5).abs() < 1e-4),
            other => panic!("expected driving, got {other:?}"),
        }
        link.report(Instant::now());
        let n = client.recv(&mut answer).unwrap();
        assert_eq!(&answer[..5], b"DKRB\x01");
        let report: RobotReport = serde_json::from_slice(&answer[5..n]).unwrap();
        assert!(
            report.health.is_none() && report.health_error.is_some(),
            "{report:?}"
        );

        std::thread::sleep(Duration::from_millis(200));
        assert!(matches!(link.step(Instant::now()), Step::Gone));
        assert!(matches!(link.step(Instant::now()), Step::Absent));
    }

    fn link() -> (Link, u16, tempfile::TempDir) {
        let dir = tempfile::tempdir().unwrap();
        let link =
            Link::bind(0, Duration::from_millis(150), &dir.path().join("none.sock")).unwrap();
        let port = link.local_addr().unwrap().port();
        (link, port, dir)
    }

    /// Sends left stick 0.5 at 50 Hz for `send_for`, after `after`.
    fn sender(port: u16, after: Duration, send_for: Duration) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            std::thread::sleep(after);
            let client = UdpSocket::bind("127.0.0.1:0").unwrap();
            let end = Instant::now() + send_for;
            let mut seq = 0;
            while Instant::now() < end {
                seq += 1;
                let p = Packet::from_axes(seq, 0, [0.0, 0.5, 0.0, 0.0], [0.0; 2], Buttons::NONE);
                client.send_to(&p.encode(), ("127.0.0.1", port)).unwrap();
                std::thread::sleep(Duration::from_millis(20));
            }
        })
    }

    /// Steps and waits like the pad loop until `Gone`, returning every stick it was handed.
    fn drive_until_gone(link: &mut Link) -> Vec<f32> {
        let mut sticks = Vec::new();
        for _ in 0..40 {
            match link.step(Instant::now()) {
                Step::Driving(f) => sticks.push(f.value(Axis::LeftStickY)),
                Step::Gone => return sticks,
                Step::Absent => panic!("gone without saying so"),
            }
            link.wait(Instant::now() + Duration::from_millis(20), false);
        }
        panic!("never gone: {sticks:?}");
    }

    /// The loop held up past the timeout — a slow answer from robotd — while the client kept
    /// sending: what queued meanwhile is read before silence is judged, so the pad is still
    /// driving rather than gone (which would reset the modes and turn the next Start into a HOME).
    /// Its sticks wait for one datagram heard live, then drive again.
    #[test]
    fn a_stalled_loop_is_not_a_client_that_went_away() {
        let (mut link, port, _dir) = link();
        let tx = sender(port, Duration::ZERO, Duration::from_millis(800));
        link.wait(Instant::now() + Duration::from_secs(1), true);
        assert!(matches!(link.step(Instant::now()), Step::Driving(_)));
        link.wait(Instant::now() + Duration::from_millis(20), false);
        std::thread::sleep(Duration::from_millis(400)); // the stall: nothing read meanwhile
        match link.step(Instant::now()) {
            Step::Driving(f) => {
                assert!(!f.new_session, "the same session");
                assert!(f.unverified);
                assert_eq!(
                    f.value(Axis::LeftStickY),
                    0.0,
                    "queued sticks are not trusted"
                );
            }
            other => panic!("a stall read as {other:?}"),
        }
        assert_eq!(link.stalls, 1, "said once");
        link.wait(Instant::now() + Duration::from_millis(60), false);
        match link.step(Instant::now()) {
            Step::Driving(f) => assert!(
                !f.unverified && (f.value(Axis::LeftStickY) - 0.5).abs() < 1e-3,
                "heard live: drives"
            ),
            other => panic!("expected driving, got {other:?}"),
        }
        tx.join().unwrap();
    }

    /// The client sent through the first part of the stall and then went away: its queued
    /// datagrams must not walk the robot again after the stall, and it is gone a timeout later.
    #[test]
    fn a_client_that_left_during_a_stall_does_not_drive_after_it() {
        let (mut link, port, _dir) = link();
        let tx = sender(port, Duration::ZERO, Duration::from_millis(250));
        link.wait(Instant::now() + Duration::from_secs(1), true);
        assert!(matches!(link.step(Instant::now()), Step::Driving(_)));
        link.wait(Instant::now() + Duration::from_millis(20), false);
        std::thread::sleep(Duration::from_millis(500)); // stall; the client stops at ~250 ms
        tx.join().unwrap();
        let sticks = drive_until_gone(&mut link);
        assert!(
            sticks.iter().all(|&v| v == 0.0),
            "a stale stick: {sticks:?}"
        );
    }

    /// A client that stops sending, with the loop running normally, is a disconnect and nothing
    /// else: no hold-up said, no stick zeroed while it was still driving.
    #[test]
    fn a_normal_disconnect_is_not_a_hold_up() {
        let (mut link, port, _dir) = link();
        let tx = sender(port, Duration::ZERO, Duration::from_millis(300));
        link.wait(Instant::now() + Duration::from_secs(1), true);
        let sticks = drive_until_gone(&mut link);
        tx.join().unwrap();
        assert!(
            sticks.len() > 5 && sticks.iter().all(|&v| (v - 0.5).abs() < 1e-3),
            "{sticks:?}"
        );
        assert_eq!(link.stalls, 0);
    }

    /// Nobody driving, the loop held up (asking robotd to sit the robot, say), and a client that
    /// came and went inside the hold-up: its queued datagrams start a session — they are real —
    /// but never with its sticks, and it is gone a timeout later.
    #[test]
    fn a_client_that_came_and_went_during_a_stall_never_drives() {
        let (mut link, port, _dir) = link();
        link.wait(Instant::now() + Duration::from_millis(20), true);
        let tx = sender(port, Duration::from_millis(50), Duration::from_millis(200));
        std::thread::sleep(Duration::from_millis(500)); // the stall
        tx.join().unwrap();
        link.wait(Instant::now() + Duration::from_millis(100), true); // the idle wait
        let first = match link.step(Instant::now()) {
            Step::Driving(f) => f,
            other => panic!("expected the queued session, got {other:?}"),
        };
        assert!(first.new_session && first.unverified);
        assert_eq!(first.value(Axis::LeftStickY), 0.0);
        link.wait(Instant::now() + Duration::from_millis(20), false);
        let sticks = drive_until_gone(&mut link);
        assert!(
            sticks.iter().all(|&v| v == 0.0),
            "a stale stick: {sticks:?}"
        );
        assert_eq!(link.stalls, 1);
    }
}
