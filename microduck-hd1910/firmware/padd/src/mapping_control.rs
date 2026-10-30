use super::*;
pub fn sync(
    input: &mut PadInput,
    continuous: &mut Continuous,
    stream: &mut UnixStream,
    changed: bool,
    blocked: bool,
    publish: impl FnOnce() -> std::io::Result<()>,
) -> std::io::Result<bool> {
    let blocked = pause(input, continuous, stream, changed, blocked)?;
    publish()?;
    Ok(blocked)
}
pub fn pause(
    input: &mut PadInput,
    continuous: &mut Continuous,
    stream: &mut UnixStream,
    changed: bool,
    blocked: bool,
) -> std::io::Result<bool> {
    if changed {
        input.reset();
        input.select = SelectButton::default();
        input.pose_reset_pending = false;
        input.buttons = ReleaseGate::default();
        continuous.at = None;
        if blocked {
            // Clear every latch once before acknowledging learning. This is intentional suspension,
            // so no disconnect/sit path or button action is allowed to run below this gate.
            for call in [
                proto::Call::RobotMove(Default::default()),
                proto::Call::RobotPose(proto::PoseParams {
                    active: false,
                    ..Default::default()
                }),
                proto::Call::RobotHead(Default::default()),
                proto::Call::RobotMouth(proto::MouthParams { open: 0.0 }),
                proto::Call::RobotSound(proto::SoundParams {
                    tag: proto::SoundTag::Wheee,
                    hold: Some(false),
                }),
            ] {
                notify(stream, &call)?;
            }
        }
    }
    Ok(blocked)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn main_loop_pause_clears_latches_once_and_bypasses_every_action_path() {
        let (mut client, server) = UnixStream::pair().unwrap();
        server
            .set_read_timeout(Some(Duration::from_millis(20)))
            .unwrap();
        let mut input = PadInput {
            mode: Mode::BodyPose,
            select: SelectButton {
                held_since: Some(Instant::now() - Duration::from_secs(5)),
                shutdown_sent: true,
            },
            prev_rt: 1.0,
            prev_lt: 1.0,
            ..Default::default()
        };
        let mut continuous = Continuous::default();
        assert!(pause(&mut input, &mut continuous, &mut client, true, true).unwrap());
        assert_eq!(input.mode, Mode::Drive);
        assert!(input.select.held_since.is_none());
        assert_eq!(input.prev_rt, 0.0);
        assert_eq!(input.prev_lt, 0.0);
        assert!(!input.select.shutdown_sent);
        let mut reader = BufReader::new(server);
        let mut methods = Vec::new();
        for _ in 0..5 {
            let mut line = String::new();
            reader.read_line(&mut line).unwrap();
            let v: serde_json::Value = serde_json::from_str(&line).unwrap();
            if v["method"] == "robot.pose" {
                assert_eq!(v["params"]["active"], false);
            }
            if v["method"] == "robot.mouth" {
                assert_eq!(v["params"]["open"], 0.0);
            }
            if v["method"] == "robot.sound" {
                assert_eq!(v["params"]["hold"], false);
            }
            methods.push(v["method"].as_str().unwrap().to_owned());
        }
        assert_eq!(
            methods,
            vec![
                "robot.move",
                "robot.pose",
                "robot.head",
                "robot.mouth",
                "robot.sound"
            ]
        );
        assert!(pause(&mut input, &mut continuous, &mut client, false, true).unwrap());
        let mut line = String::new();
        assert!(reader.read_line(&mut line).is_err());
        assert!(!pause(&mut input, &mut continuous, &mut client, true, false).unwrap());
    }
    #[test]
    fn failed_voice_state_publication_still_clears_outputs_before_returning_error() {
        let (mut client, server) = UnixStream::pair().unwrap();
        server
            .set_read_timeout(Some(Duration::from_millis(20)))
            .unwrap();
        let mut input = PadInput::default();
        let mut continuous = Continuous::default();
        let result = sync(&mut input, &mut continuous, &mut client, true, true, || {
            Err(std::io::Error::new(
                std::io::ErrorKind::PermissionDenied,
                "fixture publication denied",
            ))
        });
        assert!(result.is_err());
        let mut reader = BufReader::new(server);
        for _ in 0..5 {
            let mut line = String::new();
            assert!(reader.read_line(&mut line).unwrap() > 0);
            let call: serde_json::Value = serde_json::from_str(&line).unwrap();
            assert!(call["method"].as_str().unwrap().starts_with("robot."));
        }
    }
}
