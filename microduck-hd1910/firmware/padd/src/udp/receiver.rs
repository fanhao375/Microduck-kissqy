// Added 2026-10 by fanhao375 (microduck-replica): UDP pad ([netpad]) and pad-loss sit; ported from Pollen Robotics microduck 0.17.1 netpadd/src/receiver.rs (Apache-2.0); see IMU-MODULE.md at the repository root.
//! What arrives on the socket, turned into "this is the pad now" — with no I/O and no clock of its
//! own, so every rule below is a unit test rather than a timing experiment.
//!
//! Upstream's `netpadd/src/receiver.rs` (Pollen microduck 0.17.1), kept rule for rule: one client
//! holds the robot until it has been quiet `timeout`; anything not newer than what is held is
//! stale; a client's first datagram makes no edges. What differs is the clock. Upstream's daemon
//! wakes on each datagram and sends at most `max_hz`; `padd` reads its pad once per `--hz` tick
//! whichever source it is, so here the newest state is simply *taken* each tick, with the edges
//! accumulated since the last take.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use super::wire::{Buttons, Packet, WireError};

/// How far ahead of the last accepted `seq` a datagram may be and still count as newer. A client
/// sends at most 100 a second and is gone after `timeout` of silence, so an honest one never gets
/// near it; without it, one forged datagram far ahead would read every real one as stale and hand
/// the robot to whoever forged it.
pub const MAX_SEQ_AHEAD: u32 = 1000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Accept {
    /// Accepted, and it is the first since the client was last gone.
    Connected,
    Taken,
    /// Not newer than what is held, or further ahead than [`MAX_SEQ_AHEAD`].
    Stale,
    /// Another sender holds the lock.
    OtherPeer,
    Malformed(WireError),
}

/// A button going down or coming up, in the order it happened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    Pressed(Buttons),
    Released(Buttons),
}

/// The pad as of one take: what is held, the sticks and triggers, and the edges since the last
/// take in an order that leads from the last take's state to this one.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub peer: SocketAddr,
    /// What was held at the last take — or, for a session's first take, in the datagram that
    /// started it. Where [`Frame::edges`] start from, and so the baseline for gating a press that
    /// was already down before this robot saw it.
    pub before: Buttons,
    pub held: Buttons,
    /// `[lx, ly, rx, ry, lt, rt]`, sticks -1..1 up positive, triggers 0..1.
    pub values: [f64; 6],
    pub edges: Vec<Edge>,
    /// The first take of this client's session: what it holds was pressed before it was seen.
    pub new_session: bool,
    /// What this take was read from had queued while the pad loop was held up, for an unknown
    /// time: the sticks are centred (the triggers are not — they move nothing) until a datagram
    /// from the client is read as it lands.
    pub unverified: bool,
}

#[derive(Debug)]
pub struct Receiver {
    timeout: Duration,
    /// The client, while it is alive. `None` means nobody is driving.
    peer: Option<SocketAddr>,
    latest: Packet,
    last_rx: Instant,
    pressed: Buttons,
    released: Buttons,
    /// What was held at the last take — where this take's edges start from.
    taken_held: Buttons,
    new_session: bool,
    /// See [`Frame::unverified`].
    unverified: bool,
    reserved_seen: u16,
}

impl Receiver {
    pub fn new(timeout: Duration) -> Self {
        Self {
            timeout,
            peer: None,
            latest: Packet::default(),
            last_rx: Instant::now(),
            pressed: Buttons::NONE,
            released: Buttons::NONE,
            taken_held: Buttons::NONE,
            new_session: false,
            unverified: false,
            reserved_seen: 0,
        }
    }

    pub fn on_datagram(&mut self, now: Instant, from: SocketAddr, bytes: &[u8]) -> Accept {
        let packet = match Packet::decode(bytes) {
            Ok(p) => p,
            Err(e) => return Accept::Malformed(e),
        };
        self.reserved_seen |= packet.reserved();
        let connected = match self.peer {
            Some(peer) if peer != from => return Accept::OtherPeer,
            Some(_) => {
                // Newer, in wrapping order, and not implausibly so.
                let ahead = packet.seq.wrapping_sub(self.latest.seq);
                if ahead == 0 || ahead > MAX_SEQ_AHEAD {
                    return Accept::Stale;
                }
                let (now_held, before) = (packet.held(), self.latest.held());
                self.pressed |= now_held.newly_set(before);
                self.released |= before.newly_set(now_held);
                false
            }
            None => {
                // A new client, or the old one back: what it holds was pressed before we saw it,
                // and firing it now would run a skill nobody just asked for.
                self.peer = Some(from);
                self.pressed = Buttons::NONE;
                self.released = Buttons::NONE;
                self.taken_held = packet.held();
                self.new_session = true;
                self.unverified = false;
                true
            }
        };
        self.latest = packet;
        self.last_rx = now;
        if connected {
            Accept::Connected
        } else {
            Accept::Taken
        }
    }

    /// The client has been quiet `timeout`: it is gone, and the lock is free. True once, on the
    /// transition.
    pub fn expire(&mut self, now: Instant) -> bool {
        if self.peer.is_some() && now.duration_since(self.last_rx) >= self.timeout {
            self.peer = None;
            self.unverified = false;
            return true;
        }
        false
    }

    /// The datagram just accepted had queued while the pad loop was held up past `timeout`: it may
    /// be a client's last words from long before, and its sticks must not walk a robot the deadman
    /// had already stopped. It still keeps (or starts) the session — a slow robotd is not a lost
    /// pad — but the sticks read as centred until [`Self::live`].
    pub fn unverified(&mut self) {
        if self.peer.is_some() {
            self.unverified = true;
        }
    }

    /// A datagram from the driving client was read as it landed: the client is there now.
    pub fn live(&mut self) {
        self.unverified = false;
    }

    /// The pad now, and the edges since the last take — `None` while nobody is driving.
    pub fn take(&mut self) -> Option<Frame> {
        let peer = self.peer?;
        let held = self.latest.held();
        let before = self.taken_held;
        let edges = edges(before, held, self.pressed, self.released);
        self.taken_held = held;
        self.pressed = Buttons::NONE;
        self.released = Buttons::NONE;
        let mut values = self.latest.values();
        if self.unverified {
            values[..4].fill(0.0);
        }
        Some(Frame {
            peer,
            before,
            held,
            values,
            edges,
            new_session: std::mem::take(&mut self.new_session),
            unverified: self.unverified,
        })
    }

    pub fn peer(&self) -> Option<SocketAddr> {
        self.peer
    }

    /// A client has started driving and its first take has not happened yet.
    pub fn session_pending(&self) -> bool {
        self.peer.is_some() && self.new_session
    }

    /// Button bits this build does not know, OR-ed over everything accepted — for one log line.
    pub fn reserved_bits_seen(&self) -> u16 {
        self.reserved_seen
    }
}

/// The edges per button that lead from `before` to `now`, given that `pressed` and `released`
/// collected every rising and falling edge in between. Exact up to how many times a button was
/// tapped inside one tick, which nobody does on purpose.
fn edges(before: Buttons, now: Buttons, pressed: Buttons, released: Buttons) -> Vec<Edge> {
    let mut out = Vec::new();
    for bit in 0..13 {
        let b = Buttons(1 << bit);
        let (was, is) = (before.contains(b), now.contains(b));
        let (p, r) = (pressed.contains(b), released.contains(b));
        match (was, is) {
            (false, true) if r => {
                out.extend([Edge::Pressed(b), Edge::Released(b), Edge::Pressed(b)])
            }
            (false, true) => out.push(Edge::Pressed(b)),
            (true, false) if p => {
                out.extend([Edge::Released(b), Edge::Pressed(b), Edge::Released(b)])
            }
            (true, false) => out.push(Edge::Released(b)),
            (true, true) if p || r => out.extend([Edge::Released(b), Edge::Pressed(b)]),
            (false, false) if p || r => out.extend([Edge::Pressed(b), Edge::Released(b)]),
            _ => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: &str = "10.0.0.2:5000";
    const B: &str = "10.0.0.3:5000";

    fn rx() -> Receiver {
        Receiver::new(Duration::from_millis(250))
    }
    fn addr(s: &str) -> SocketAddr {
        s.parse().unwrap()
    }
    fn pkt(seq: u32, buttons: Buttons, lx: f64) -> [u8; 28] {
        Packet::from_axes(seq, 0, [lx, 0.0, 0.0, 0.0], [0.0, 0.0], buttons).encode()
    }
    fn ms(t0: Instant, n: u64) -> Instant {
        t0 + Duration::from_millis(n)
    }

    #[test]
    fn a_first_datagram_is_a_new_session_with_no_edges() {
        let (mut r, t) = (rx(), Instant::now());
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(1, Buttons::A | Buttons::UP, 0.5)),
            Accept::Connected
        );
        let f = r.take().unwrap();
        assert!(f.new_session);
        assert!(f.edges.is_empty(), "held before we saw it: {:?}", f.edges);
        assert!(f.held.contains(Buttons::A));
        assert!((f.values[0] - 0.5).abs() < 1e-4);
        assert!(!r.take().unwrap().new_session, "once");
    }

    #[test]
    fn the_newest_state_wins_and_a_late_datagram_is_dropped() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, 0.5));
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(4, Buttons::NONE, -0.5)),
            Accept::Stale
        );
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, -0.5)),
            Accept::Stale
        );
        r.on_datagram(t, addr(A), &pkt(6, Buttons::NONE, 0.2));
        r.on_datagram(t, addr(A), &pkt(7, Buttons::NONE, 0.3));
        assert!((r.take().unwrap().values[0] - 0.3).abs() < 1e-4);
    }

    #[test]
    fn seq_wraps() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(u32::MAX, Buttons::NONE, 0.0));
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(0, Buttons::NONE, 0.0)),
            Accept::Taken
        );
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(u32::MAX, Buttons::NONE, 0.0)),
            Accept::Stale
        );
    }

    #[test]
    fn a_datagram_too_far_ahead_is_dropped() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(5, Buttons::NONE, 0.5));
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(5 + MAX_SEQ_AHEAD + 1, Buttons::NONE, -1.0)),
            Accept::Stale
        );
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(6, Buttons::NONE, 0.5)),
            Accept::Taken
        );
        assert_eq!(
            r.on_datagram(t, addr(A), &pkt(6 + MAX_SEQ_AHEAD, Buttons::NONE, 0.5)),
            Accept::Taken,
            "the bound itself is accepted"
        );
    }

    /// A press and a release that both land between two takes still make a tap — Start pressed
    /// and let go inside one tick must still stand the robot up, and Select's must still stop it.
    #[test]
    fn a_tap_inside_one_tick_is_both_edges_in_order() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        r.take();
        r.on_datagram(ms(t, 5), addr(A), &pkt(2, Buttons::START, 0.0));
        r.on_datagram(ms(t, 10), addr(A), &pkt(3, Buttons::NONE, 0.0));
        let f = r.take().unwrap();
        assert_eq!(
            f.edges,
            [
                Edge::Pressed(Buttons::START),
                Edge::Released(Buttons::START)
            ]
        );
        assert!(r.take().unwrap().edges.is_empty(), "edges are consumed");
    }

    #[test]
    fn a_hold_is_one_press_then_one_release() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        r.take();
        r.on_datagram(t, addr(A), &pkt(2, Buttons::SELECT, 0.0));
        assert_eq!(r.take().unwrap().edges, [Edge::Pressed(Buttons::SELECT)]);
        r.on_datagram(t, addr(A), &pkt(3, Buttons::SELECT, 0.0));
        assert!(r.take().unwrap().edges.is_empty());
        r.on_datagram(t, addr(A), &pkt(4, Buttons::NONE, 0.0));
        assert_eq!(r.take().unwrap().edges, [Edge::Released(Buttons::SELECT)]);
    }

    /// Let go and pressed again between two takes: the order leads from held to held, so the
    /// release is not lost under the press.
    #[test]
    fn a_re_press_inside_one_tick_is_release_then_press() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.0));
        r.take();
        r.on_datagram(t, addr(A), &pkt(2, Buttons::X, 0.0));
        r.take();
        r.on_datagram(t, addr(A), &pkt(3, Buttons::NONE, 0.0));
        r.on_datagram(t, addr(A), &pkt(4, Buttons::X, 0.0));
        assert_eq!(
            r.take().unwrap().edges,
            [Edge::Released(Buttons::X), Edge::Pressed(Buttons::X)]
        );
    }

    #[test]
    fn another_peer_is_ignored_while_the_first_is_alive() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.5));
        assert_eq!(
            r.on_datagram(ms(t, 10), addr(B), &pkt(99, Buttons::NONE, -1.0)),
            Accept::OtherPeer
        );
        assert!((r.take().unwrap().values[0] - 0.5).abs() < 1e-4);
    }

    #[test]
    fn going_quiet_expires_once_and_frees_the_lock() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::START, 0.5));
        assert!(!r.expire(ms(t, 249)));
        assert!(r.expire(ms(t, 250)));
        assert!(!r.expire(ms(t, 300)), "once");
        assert!(r.take().is_none(), "nobody is driving");
        assert_eq!(
            r.on_datagram(ms(t, 400), addr(B), &pkt(1, Buttons::NONE, 0.0)),
            Accept::Connected
        );
    }

    #[test]
    fn a_restarted_client_is_taken_after_the_timeout() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1000, Buttons::NONE, 0.0));
        assert_eq!(
            r.on_datagram(ms(t, 50), addr(A), &pkt(3, Buttons::NONE, 0.0)),
            Accept::Stale
        );
        assert!(r.expire(ms(t, 250)));
        assert_eq!(
            r.on_datagram(ms(t, 260), addr(A), &pkt(4, Buttons::NONE, 0.0)),
            Accept::Connected
        );
    }

    /// Start held when the client drops and still held when it comes back: the new session makes
    /// no edge for it, so the release that follows has no press to pair with.
    #[test]
    fn start_held_across_a_dropout_is_not_a_new_press() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::START, 0.0));
        r.take();
        assert!(r.expire(ms(t, 300)));
        r.on_datagram(ms(t, 1_000), addr(A), &pkt(2, Buttons::START, 0.0));
        let f = r.take().unwrap();
        assert!(f.new_session && f.edges.is_empty());
        r.on_datagram(ms(t, 1_050), addr(A), &pkt(3, Buttons::NONE, 0.0));
        assert_eq!(r.take().unwrap().edges, [Edge::Released(Buttons::START)]);
    }

    /// Select held in the datagram that starts a session and let go before the first take: the
    /// release is an edge, and `before` says the button was already down — the baseline the pad
    /// loop gates it with, so it cannot become the emergency-stop release.
    #[test]
    fn a_session_first_take_starts_from_the_connecting_datagram() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::SELECT, 0.0));
        assert!(r.session_pending());
        r.on_datagram(ms(t, 50), addr(A), &pkt(2, Buttons::NONE, 0.0));
        let f = r.take().unwrap();
        assert!(f.new_session && !r.session_pending());
        assert_eq!(f.before, Buttons::SELECT);
        assert_eq!(f.held, Buttons::NONE);
        assert_eq!(f.edges, [Edge::Released(Buttons::SELECT)]);
        assert_eq!(
            r.take().unwrap().before,
            Buttons::NONE,
            "then from each take"
        );
    }

    /// Read after the loop was held up, a datagram's sticks are of unknown age: centred until one
    /// is heard live. Its buttons and triggers still count, and the session goes on.
    #[test]
    fn sticks_read_after_a_hold_up_stay_centred_until_heard_live() {
        let (mut r, t) = (rx(), Instant::now());
        r.unverified();
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.8));
        assert!(
            !r.take().unwrap().unverified,
            "nobody was driving: nothing to mark"
        );
        let p = Packet::from_axes(2, 0, [0.8, 0.8, 0.8, 0.8], [0.6, 0.7], Buttons::START);
        r.on_datagram(ms(t, 300), addr(A), &p.encode());
        r.unverified();
        let f = r.take().unwrap();
        assert!(f.unverified && !f.new_session);
        assert_eq!(f.values[..4], [0.0; 4], "sticks centred");
        assert!(
            (f.values[4] - 0.6).abs() < 1e-4 && (f.values[5] - 0.7).abs() < 1e-4,
            "triggers kept"
        );
        assert_eq!(
            f.edges,
            [Edge::Pressed(Buttons::START)],
            "buttons still count"
        );
        assert!(r.take().unwrap().unverified, "until heard live");
        r.live();
        let f = r.take().unwrap();
        assert!(!f.unverified && (f.values[0] - 0.8).abs() < 1e-4);
    }

    /// Gone, and back as a new session from datagrams that queued during a hold-up: the mark is
    /// set after the session starts, so the new session's sticks are centred too.
    #[test]
    fn a_new_session_from_queued_datagrams_is_unverified_and_expiry_clears_the_mark() {
        let (mut r, t) = (rx(), Instant::now());
        r.on_datagram(t, addr(A), &pkt(1, Buttons::NONE, 0.5));
        r.unverified();
        assert!(r.expire(ms(t, 300)));
        assert_eq!(
            r.on_datagram(ms(t, 400), addr(B), &pkt(1, Buttons::NONE, 0.9)),
            Accept::Connected
        );
        let f = r.take().unwrap();
        assert!(!f.unverified, "expiry cleared the old mark");
        assert!(r.expire(ms(t, 700)));
        r.on_datagram(ms(t, 800), addr(B), &pkt(9, Buttons::NONE, 0.9));
        r.unverified();
        let f = r.take().unwrap();
        assert!(
            f.new_session && f.unverified && f.values[0] == 0.0,
            "lx, where pkt puts 0.9"
        );
    }

    #[test]
    fn malformed_changes_nothing_and_takes_no_lock() {
        let (mut r, t) = (rx(), Instant::now());
        assert!(matches!(
            r.on_datagram(t, addr(A), b"hello"),
            Accept::Malformed(_)
        ));
        assert!(r.take().is_none());
        assert_eq!(
            r.on_datagram(t, addr(B), &pkt(1, Buttons::NONE, 0.0)),
            Accept::Connected
        );
    }
}
