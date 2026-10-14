// Modified 2026-10 by fanhao375 (microduck-replica): optional servo P/D/torque-limit file (servo-test-parameters.json); see IMU-MODULE.md at the repository root.
//! Optional servo P/D/torque limit for HD1910 (servo-test-parameters.json next to the
//! calibration; written by packaging/service.sh servo-parameters). No EEPROM writes or
//! model changes. Missing file leaves the V167 arm path in charge.
//! Ranges are narrower than the registers allow: P matches the model-contract kp_fw
//! check (1..=32), D at most 64, torque limit 200..=1000 so a typo cannot leave a leg limp.
use crate::feetech::Bus;
use crate::hd1910::RegisterSnapshot;
use crate::io::{IoError, Result};
use serde::{Deserialize, Serialize};
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub kp: u8,
    pub kd: u8,
    pub torque_limit: u16,
}

pub fn load(calibration_path: &Path) -> Result<Option<Parameters>> {
    let path = calibration_path.with_file_name("servo-test-parameters.json");
    let bytes = match std::fs::read(path) {
        Ok(bytes) => bytes,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(IoError::Bus(format!("servo test parameters: {e}"))),
    };
    let parameters: Parameters = serde_json::from_slice(&bytes)
        .map_err(|e| IoError::Bus(format!("servo test parameters: {e}")))?;
    if !(1..=32).contains(&parameters.kp) {
        return Err(IoError::Bus("servo test P must be 1..=32".into()));
    }
    if parameters.kd > 64 {
        return Err(IoError::Bus("servo test D must be 0..=64".into()));
    }
    if !(200..=1000).contains(&parameters.torque_limit) {
        return Err(IoError::Bus(
            "servo test torque limit must be 200..=1000".into(),
        ));
    }
    Ok(Some(parameters))
}

fn payload(kp: u8, kd: u8, torque_limit: u16) -> Vec<u8> {
    let [lo, hi] = torque_limit.to_le_bytes();
    vec![lo, hi, kp, kd]
}

pub(crate) fn rows(snapshots: &[RegisterSnapshot], parameters: Parameters) -> Vec<(u8, Vec<u8>)> {
    snapshots
        .iter()
        .map(|s| {
            (
                s.id,
                payload(parameters.kp, parameters.kd, parameters.torque_limit),
            )
        })
        .collect()
}

/// Relax rows: P as inspected before arm, D=0, torque limit 1000 — the RAM state V167
/// leaves after relax, so relaxing looks the same with or without the file.
pub(crate) fn relax_rows(snapshots: &[RegisterSnapshot]) -> Vec<(u8, Vec<u8>)> {
    snapshots
        .iter()
        .map(|s| (s.id, payload(s.temporary_pdi[0], 0, 1000)))
        .collect()
}

pub(crate) fn verify(bus: &mut Bus, rows: &[(u8, Vec<u8>)]) -> Result<()> {
    let ids: Vec<_> = rows.iter().map(|(id, _)| *id).collect();
    let actual = bus.sync_read(48, 4, &ids)?;
    for (id, expected) in rows {
        if actual.get(id) != Some(expected) {
            return Err(IoError::Bus(format!(
                "ID {id}: servo test P/D/torque limit readback mismatch"
            )));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_an_explicit_saved_test_file_activates_the_extension() {
        let name = format!(
            "servo-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        );
        let root = std::env::temp_dir().join(name);
        std::fs::create_dir(&root).unwrap();
        let calibration = root.join("hd1910-calibration.toml");
        assert_eq!(load(&calibration).unwrap(), None);
        let path = root.join("servo-test-parameters.json");
        std::fs::write(&path, r#"{"kp":3,"kd":0,"torque_limit":681}"#).unwrap();
        assert_eq!(
            load(&calibration).unwrap(),
            Some(Parameters {
                kp: 3,
                kd: 0,
                torque_limit: 681
            })
        );
        for invalid in [
            r#"{"kp":256,"kd":0,"torque_limit":681}"#,
            r#"{"kp":3,"kd":0,"torque_limit":1001}"#,
            r#"{"kp":3,"kd":0}"#,
            r#"{"kp":0,"kd":0,"torque_limit":681}"#,
            r#"{"kp":33,"kd":0,"torque_limit":681}"#,
            r#"{"kp":3,"kd":65,"torque_limit":681}"#,
            r#"{"kp":3,"kd":0,"torque_limit":199}"#,
            r#"{"kp":3,"kd":0,"torque_limit":681,"extra":1}"#,
            r#"not json"#,
        ] {
            std::fs::write(&path, invalid).unwrap();
            assert!(load(&calibration).is_err());
        }
        std::fs::remove_file(&path).unwrap();
        assert_eq!(load(&calibration).unwrap(), None);
        std::fs::remove_dir(root).unwrap();
    }

    #[test]
    fn rows_are_limit_p_d_and_relax_rows_keep_p_with_d0_limit1000() {
        let snapshot = |id, p| RegisterSnapshot {
            id,
            name: String::new(),
            mode: 4,
            eeprom_pdi: [32, 40, 0],
            temporary_pdi: [p, 40, 9],
            acceleration_raw: 64,
            goal_current_raw: 0,
            goal_speed_raw: 0,
        };
        let snapshots = [snapshot(20, 32), snapshot(21, 5)];
        let parameters = Parameters {
            kp: 3,
            kd: 1,
            torque_limit: 681,
        };
        assert_eq!(
            rows(&snapshots, parameters),
            vec![(20, vec![0xa9, 0x02, 3, 1]), (21, vec![0xa9, 0x02, 3, 1])]
        );
        // Relax: P from before arm, D=0, limit 1000 (V167's post-relax RAM), I untouched.
        assert_eq!(
            relax_rows(&snapshots),
            vec![(20, vec![0xe8, 0x03, 32, 0]), (21, vec![0xe8, 0x03, 5, 0])]
        );
    }
}
