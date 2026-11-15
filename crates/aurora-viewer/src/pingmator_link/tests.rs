use super::*;

#[test]
fn parses_the_messages() {
    assert_eq!(parse(b"\n"), Ok(Message::Heartbeat));
    assert_eq!(
        parse(br#"{"type":"hello","app":"Pingmator"}"#),
        Ok(Message::Hello(Some("Pingmator".into())))
    );
    assert_eq!(
        parse(br#"{"type":"pose","bvh":"HIERARCHY"}"#),
        Ok(Message::Pose("HIERARCHY".into()))
    );
    assert_eq!(parse(br#"{"type":"later"}"#), Ok(Message::Unknown));
    assert!(parse(br#"{"type":"pose"}"#).is_err());
    assert!(parse(b"not json").is_err());
}

#[test]
fn a_long_app_name_is_cut() {
    let line = format!(r#"{{"type":"hello","app":"{}"}}"#, "x".repeat(500));
    assert!(matches!(parse(line.as_bytes()), Ok(Message::Hello(Some(name))) if name.len() == 64));
}

/// A pose saved by Pingmator's own test (tst_auroralink, PINGMATOR_POSE_OUT):
/// the left forearm turned by (10, 20, 30) degrees on frame 5.
const PINGMATOR_POSE: &str = include_str!("pingmator_pose.bvh");

fn skeleton() -> Skeleton {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/character/avatar_skeleton.xml");
    Skeleton::parse(&std::fs::read(path).expect("avatar_skeleton.xml")).expect("skeleton")
}

#[test]
fn a_pingmator_pose_is_read() {
    let animation = decode(PINGMATOR_POSE, &skeleton()).expect("Pingmator's pose BVH");
    let rotation = |name: &str| {
        animation
            .joints
            .iter()
            .find(|j| j.joint_name == name)
            .and_then(|j| j.rot_keys.first())
            .map(|k| k.rotation)
    };
    let elbow = rotation("mElbowLeft").expect("the left forearm is in the pose");
    assert!(elbow.angle_between(glam::Quat::IDENTITY) > 0.5, "turned: {elbow:?}");
    for still in ["mElbowRight", "mHead", "mKneeLeft"] {
        assert!(
            rotation(still).is_none_or(|q| q.angle_between(glam::Quat::IDENTITY) < 1e-3),
            "{still} stays still"
        );
    }
}

#[test]
fn a_broken_pose_is_refused() {
    assert!(decode("HIERARCHY\nnot a bvh", &skeleton()).is_err());
}

#[test]
fn the_link_greets_and_takes_a_pose() {
    use std::io::{BufRead, BufReader, Write};
    let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).unwrap();
    let port = listener.local_addr().unwrap().port();
    let shared = start_listener(listener, skeleton()).unwrap();
    let mut stream = TcpStream::connect((Ipv4Addr::LOCALHOST, port)).unwrap();
    stream.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let mut reader = BufReader::new(stream.try_clone().unwrap());
    let mut line = String::new();
    reader.read_line(&mut line).unwrap();
    let hello: serde_json::Value = serde_json::from_str(&line).unwrap();
    assert_eq!(hello["type"], "hello");
    assert_eq!(hello["protocol"], PROTOCOL);

    writeln!(stream, "{}", serde_json::json!({ "type": "hello", "app": "Pingmator" })).unwrap();
    writeln!(stream, "{}", serde_json::json!({ "type": "pose", "bvh": PINGMATOR_POSE })).unwrap();
    let start = std::time::Instant::now();
    while shared.latest.lock().is_none() && start.elapsed() < Duration::from_secs(5) {
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(shared.latest.lock().is_some(), "the pose arrived");
    assert_eq!(shared.client.lock().as_deref(), Some("Pingmator"));

    // a pose it cannot read is answered with an error, and the link stays up
    writeln!(stream, "{}", serde_json::json!({ "type": "pose", "bvh": "nonsense" })).unwrap();
    line.clear();
    reader.read_line(&mut line).unwrap();
    let error: serde_json::Value = serde_json::from_str(&line).unwrap();
    assert_eq!(error["type"], "error");
    shared.stop.store(true, Ordering::Release);
}
