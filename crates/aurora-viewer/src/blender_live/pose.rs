//! Live poses over LiveMesh (Aurora addition, no Legacy Viewer counterpart).
//! A local tool (Pingmator, a script) asks for the `pose-bvh` feature, proves
//! it can read the session key file, then sends poses as small BVH texts.
//! The viewer shows them on the user's own avatar through the local poser,
//! visible to the user only. Nothing reaches the simulator.
//!
//! Security: LiveMesh already refuses anything that does not start with its
//! opening code and caps every message. Poses also need `auth`: each time
//! LiveMesh starts, a new random key is written to a file only the current
//! user can read (`livemesh.json` in the viewer's local data folder). The
//! `hello` carries a random challenge per connection; the client answers with
//! HMAC-SHA256(key, challenge), so the key never crosses the socket. Mesh
//! snapshots without `auth` keep working as before (the Blender add-on).
//! See docs/BLENDER_LIVE.md, "Live poses".
//!
//! Two pose messages: `pose-bvh` (a small BVH, read like a Poser pose file)
//! and `pose` (local rotations as quaternions per SL joint, in the SL avatar
//! frame: X forward, Y left, Z up; the pelvis offset in metres), which skips
//! the BVH text and its Euler angles.

use aurora_assets::{
    Animation, JointMotion, Skeleton,
    anim::{PosKey, RotKey},
};
use glam::{Quat, Vec3};
use sha2::{Digest, Sha256};
use std::{io, path::PathBuf};

/// A pose message is a few tens of KB; anything above is not one.
pub(super) const MAX_MESSAGE: usize = 1024 * 1024;
const KEY_FILE: &str = "livemesh.json";

/// What the pose feature needs on the socket thread.
pub(super) struct PoseLink {
    pub key: [u8; 32],
    pub skeleton: Skeleton,
}

/// 32 random bytes from the system generator (two version-4 UUIDs: 244
/// random bits; `uuid` is already a dependency and uses the OS generator).
pub(super) fn random_bytes() -> [u8; 32] {
    let mut bytes = [0; 32];
    bytes[..16].copy_from_slice(uuid::Uuid::new_v4().as_bytes());
    bytes[16..].copy_from_slice(uuid::Uuid::new_v4().as_bytes());
    bytes
}

pub(super) fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(text: &str) -> Option<Vec<u8>> {
    if !text.len().is_multiple_of(2) {
        return None;
    }
    (0..text.len())
        .step_by(2)
        .map(|i| text.get(i..i + 2).and_then(|pair| u8::from_str_radix(pair, 16).ok()))
        .collect()
}

/// HMAC-SHA256 (RFC 2104) on the `sha2` crate already in the tree.
pub(super) fn hmac_sha256(key: &[u8], message: &[u8]) -> [u8; 32] {
    let mut block = [0u8; 64];
    if key.len() > 64 {
        block[..32].copy_from_slice(&Sha256::digest(key));
    } else {
        block[..key.len()].copy_from_slice(key);
    }
    let pad = |byte: u8| block.map(|b| b ^ byte);
    let inner = Sha256::new().chain_update(pad(0x36)).chain_update(message).finalize();
    Sha256::new().chain_update(pad(0x5c)).chain_update(inner).finalize().into()
}

/// Constant-time comparison: the time does not tell how many bytes matched.
fn same(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

/// Whether `response` (hex) is HMAC-SHA256(key, challenge).
pub(super) fn verify(key: &[u8; 32], challenge: &[u8; 32], response: &str) -> bool {
    unhex(response).is_some_and(|response| same(&response, &hmac_sha256(key, challenge)))
}

/// A message of the pose feature. Anything else goes on to the controls and
/// the mesh snapshots, as before.
#[derive(Debug, PartialEq)]
pub(super) enum Message {
    Auth {
        client: Option<String>,
        response: String,
    },
    Pose(String),
    /// `pose`: joint name -> rotation (x, y, z, w), joint name -> offset (m).
    Quaternions {
        rotations: Vec<(String, [f32; 4])>,
        positions: Vec<(String, [f32; 3])>,
    },
}

/// A pose names at most this many joints (the SL skeleton has 133 bones).
const MAX_POSE_JOINTS: usize = 256;

/// Reads a pose-feature message. `None` when the bytes are not one (the
/// caller then treats them as before); an error when they are one but
/// malformed.
pub(super) fn parse(bytes: &[u8]) -> Option<Result<Message, String>> {
    #[derive(serde::Deserialize)]
    struct Raw {
        #[serde(rename = "type")]
        kind: Option<String>,
        client: Option<String>,
        response: Option<String>,
        bvh: Option<String>,
        rotations: Option<std::collections::BTreeMap<String, [f32; 4]>>,
        positions: Option<std::collections::BTreeMap<String, [f32; 3]>>,
    }
    if bytes.len() > MAX_MESSAGE || !bytes.starts_with(b"{") {
        return None;
    }
    let raw = serde_json::from_slice::<Raw>(bytes).ok()?;
    match raw.kind.as_deref()? {
        "auth" => Some(match raw.response {
            Some(response) => Ok(Message::Auth {
                client: raw.client.map(|name| name.chars().filter(|c| !c.is_control()).take(64).collect()),
                response,
            }),
            None => Err("auth without response".into()),
        }),
        "pose-bvh" => Some(raw.bvh.map(Message::Pose).ok_or_else(|| "pose-bvh without bvh".to_string())),
        "pose" => Some(match raw.rotations {
            Some(rotations) => {
                let positions = raw.positions.unwrap_or_default();
                if rotations.len() > MAX_POSE_JOINTS || positions.len() > MAX_POSE_JOINTS {
                    Err("too many joints in pose".into())
                } else {
                    Ok(Message::Quaternions {
                        rotations: rotations.into_iter().collect(),
                        positions: positions.into_iter().collect(),
                    })
                }
            }
            None => Err("pose without rotations".into()),
        }),
        _ => None,
    }
}

/// The pose a BVH holds: its first frame, as the poser's Load reads it.
pub(super) fn decode(bvh: &str, skeleton: &Skeleton) -> Result<Animation, String> {
    aurora_assets::import_animation::parse_bvh_pose(bvh.as_bytes(), skeleton)
        .map(|b| b.animation)
        .map_err(|e| e.to_string())
}

/// The track of a joint in `joints` (created on first use), by SL name or
/// alias; `None` for a name the skeleton does not know.
fn track(joints: &mut Vec<JointMotion>, skeleton: &Skeleton, name: &str) -> Option<usize> {
    let name = &skeleton.joints[skeleton.find(name)?].name;
    if let Some(i) = joints.iter().position(|j| &j.joint_name == name) {
        return Some(i);
    }
    joints.push(JointMotion {
        joint_name: name.clone(),
        priority: 2,
        rot_keys: Vec::new(),
        pos_keys: Vec::new(),
    });
    Some(joints.len() - 1)
}

/// A `pose` message as a one-frame animation, the shape `parse_bvh_pose`
/// gives the Poser: SL joint names (aliases accepted), rotations in the SL
/// avatar frame relative to the rest pose, the pelvis offset in metres.
pub(super) fn quaternion_pose(
    rotations: &[(String, [f32; 4])],
    positions: &[(String, [f32; 3])],
    skeleton: &Skeleton,
) -> Result<Animation, String> {
    let mut joints: Vec<JointMotion> = Vec::new();
    for (name, [x, y, z, w]) in rotations {
        let q = Quat::from_xyzw(*x, *y, *z, *w);
        if !q.is_finite() || q.length_squared() < 1e-6 {
            return Err(format!("invalid rotation for {name}"));
        }
        if let Some(i) = track(&mut joints, skeleton, name) {
            joints[i].rot_keys = vec![RotKey {
                time: 0.0,
                rotation: q.normalize(),
            }];
        }
    }
    for (name, [x, y, z]) in positions {
        let p = Vec3::new(*x, *y, *z);
        if !p.is_finite() {
            return Err(format!("invalid position for {name}"));
        }
        if let Some(i) = track(&mut joints, skeleton, name) {
            joints[i].pos_keys = vec![PosKey {
                time: 0.0,
                position: p.clamp(Vec3::splat(-5.0), Vec3::splat(5.0)),
            }];
        }
    }
    if joints.is_empty() {
        return Err("pose names no known joint".into());
    }
    Ok(Animation {
        base_priority: 2,
        duration: 1.0,
        emote_name: String::new(),
        loop_in: 0.0,
        loop_out: 1.0,
        looping: false,
        ends_after: None,
        ease_in: 0.3,
        ease_out: 0.3,
        hand_pose: 1,
        joints,
    })
}

fn key_path() -> Option<PathBuf> {
    directories::ProjectDirs::from("org", "Aurora", "AuroraViewer").map(|d| d.data_local_dir().join(KEY_FILE))
}

/// The session key file; removed when LiveMesh stops (drop).
pub(super) struct KeyFile {
    path: PathBuf,
}

impl KeyFile {
    /// Writes a new key for this LiveMesh session.
    pub fn create(port: u16, key: &[u8; 32]) -> io::Result<Self> {
        let path = key_path().ok_or_else(|| io::Error::other("no local data folder"))?;
        Self::create_at(path, port, key)
    }

    pub(super) fn create_at(path: PathBuf, port: u16, key: &[u8; 32]) -> io::Result<Self> {
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        // A stale file of an earlier session would keep its old rights.
        let _ = std::fs::remove_file(&path);
        let text = serde_json::json!({
            "protocol": super::protocol::VERSION,
            "port": port,
            "key": hex(key),
            "pid": std::process::id(),
        })
        .to_string();
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create_new(true);
        // The local data folder is private to the user on Windows; on other
        // systems the file itself is made readable by its owner only.
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        use std::io::Write;
        options.open(&path)?.write_all(text.as_bytes())?;
        Ok(Self { path })
    }
}

impl Drop for KeyFile {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.path);
    }
}
