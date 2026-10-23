//! DXL 2.0 framing for the IMU sharing the native Feetech UART.
//!
//! I/O and the absolute transaction deadline belong to `feetech::Bus`. The
//! rustypot public transaction API has its own flush/timeout policy and keeps
//! its codec private, so it cannot be used inside that shared deadline.
pub(crate) const HEADER: [u8; 4] = [255, 255, 253, 0];

pub(crate) fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0u16;
    for byte in bytes {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = (crc << 1) ^ if crc & 0x8000 != 0 { 0x8005 } else { 0 };
        }
    }
    crc
}

pub(crate) fn packet(id: u8, instruction: u8, params: &[u8]) -> Vec<u8> {
    let mut body = Vec::with_capacity(params.len() + 1);
    for byte in std::iter::once(instruction).chain(params.iter().copied()) {
        body.push(byte);
        if body.ends_with(&[255, 255, 253]) {
            body.push(253);
        }
    }
    let mut raw = HEADER.to_vec();
    raw.push(id);
    raw.extend_from_slice(&((body.len() + 2) as u16).to_le_bytes());
    raw.extend_from_slice(&body);
    raw.extend_from_slice(&crc16(&raw).to_le_bytes());
    raw
}

#[derive(Debug, PartialEq)]
pub(crate) struct Status {
    pub id: u8,
    pub error: u8,
    pub data: Vec<u8>,
    pub wire_len: usize,
}

#[derive(Default)]
pub(crate) struct Decoder {
    bytes: Vec<u8>,
    pub bad_checksums: u64,
    pub discarded: u64,
}
impl Decoder {
    fn discard(&mut self, count: usize) {
        self.bytes.drain(..count);
        self.discarded += count as u64;
    }

    pub fn push(&mut self, byte: u8) -> Option<Status> {
        self.bytes.push(byte);
        loop {
            let Some(start) = self.bytes.windows(4).position(|w| w == HEADER) else {
                self.discard(self.bytes.len().saturating_sub(3));
                return None;
            };
            self.discard(start);
            if self.bytes.len() < 7 {
                return None;
            }
            let length = usize::from(u16::from_le_bytes([self.bytes[5], self.bytes[6]]));
            if !(4..=512).contains(&length) {
                self.discard(1);
                continue;
            }
            let total = 7 + length;
            if self.bytes.len() < total {
                // A stuffed body cannot contain another complete header. Do
                // not let a corrupt length hide the next real status packet.
                if let Some(next) = self.bytes[4..].windows(4).position(|w| w == HEADER) {
                    self.discard(next + 4);
                    continue;
                }
                return None;
            }
            if crc16(&self.bytes[..total - 2])
                != u16::from_le_bytes([self.bytes[total - 2], self.bytes[total - 1]])
            {
                self.bad_checksums += 1;
                self.discard(1);
                continue;
            }
            let raw = self.bytes.drain(..total).collect::<Vec<_>>();
            let mut body = Vec::new();
            let mut i = 7;
            let mut valid = true;
            while i < total - 2 {
                body.push(raw[i]);
                i += 1;
                if body.ends_with(&[255, 255, 253]) {
                    if i >= total - 2 || raw[i] != 253 {
                        valid = false;
                        break;
                    }
                    i += 1;
                }
            }
            if valid && body.len() >= 2 && body[0] == 0x55 {
                return Some(Status {
                    id: raw[4],
                    error: body[1],
                    data: body[2..].to_vec(),
                    wire_len: total,
                });
            }
            self.discarded += total as u64;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn instruction_bytes_match_robotis_and_the_servo_web_adapter() {
        assert_eq!(
            packet(1, 1, &[]),
            [255, 255, 253, 0, 1, 3, 0, 1, 0x19, 0x4e]
        );
        assert_eq!(
            packet(1, 2, &[0x84, 0, 4, 0]),
            [255, 255, 253, 0, 1, 7, 0, 2, 0x84, 0, 4, 0, 0x1d, 0x15]
        );
    }

    #[test]
    fn fragmented_stuffed_status_recovers_the_original_payload() {
        let payload = [1, 255, 255, 253, 253, 2, 255, 255, 253, 3];
        let frame = packet(200, 0x55, &[&[0][..], &payload].concat());
        for split in 0..=frame.len() {
            let mut decoder = Decoder::default();
            let mut statuses = Vec::new();
            for chunk in [&frame[..split], &frame[split..]] {
                statuses.extend(chunk.iter().filter_map(|b| decoder.push(*b)));
            }
            assert_eq!(
                statuses,
                [Status {
                    id: 200,
                    error: 0,
                    data: payload.to_vec(),
                    wire_len: frame.len()
                }]
            );
        }
    }

    #[test]
    fn noise_crc_errors_and_a_truncated_length_do_not_hide_the_next_packet() {
        let frame = packet(200, 0x55, &[0, 1, 2, 3]);
        let mut broken = frame.clone();
        *broken.last_mut().unwrap() ^= 1;
        let prefix = [255, 255, 253, 0, 200, 60, 0, 0x55];
        let input = [&b"noise"[..], &broken, &prefix, &frame].concat();
        let mut decoder = Decoder::default();
        let statuses = input
            .iter()
            .filter_map(|b| decoder.push(*b))
            .collect::<Vec<_>>();
        assert_eq!(
            statuses,
            [Status {
                id: 200,
                error: 0,
                data: vec![1, 2, 3],
                wire_len: frame.len()
            }]
        );
        assert!(decoder.bad_checksums > 0);
        assert!(decoder.discarded > 0);
    }

    #[test]
    fn instruction_echo_is_not_a_status_packet() {
        let mut decoder = Decoder::default();
        assert!(
            packet(200, 2, &[124, 0, 12, 0])
                .into_iter()
                .all(|b| decoder.push(b).is_none())
        );
    }
}
