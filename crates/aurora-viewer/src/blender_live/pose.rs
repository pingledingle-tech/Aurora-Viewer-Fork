//! Live poses over LiveMesh (Aurora addition, no Legacy Viewer counterpart).
//! A local tool (PinglyMotion, a script) asks for the `pose-bvh` feature, proves
//! it can read the session's token file (`token.rs`), then sends poses as
//! small BVH texts. It greets with `AURPOS1` instead of the add-on's `AURBLD2`.
//! The viewer shows them on the user's own avatar through the local poser,
//! visible to the user only. Nothing reaches the simulator.
//!
//! Security: LiveMesh already refuses anything that does not start with its
//! opening code and caps every message. Poses also need `auth`: the key is
//! the session secret that LiveMesh writes, new at each start, to the token
//! file only the current user can read (`<port>.token`, `token.rs`). The
//! `hello` carries a random challenge per connection; the client answers with
//! HMAC-SHA256(key, challenge), so the key never crosses the socket. Mesh
//! snapshots without `auth` keep working as before (the Blender add-on).
//! The proof also runs the other way (`auth-mutual`): a client that sends a
//! random `nonce` with its `auth` gets back `server_proof`, an HMAC of that
//! nonce under the same key, so it can tell that the program on the port
//! reads the key file, as Aurora does, before it sends any pose.
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

/// A pose message is a few tens of KB; anything above is not one.
pub(super) const MAX_MESSAGE: usize = 1024 * 1024;

/// What the pose feature needs on the socket thread.
pub(super) struct PoseLink {
    pub key: [u8; 32],
    pub skeleton: Skeleton,
}

impl PoseLink {
    /// The key of the handshake is the preview's session secret (`token.rs`,
    /// 64 hex characters = 32 bytes), so there is one key file for the Blender
    /// add-on and for the pose tools. `None` when it is not that shape.
    pub(super) fn new(secret: &str, skeleton: Skeleton) -> Option<Self> {
        let key = unhex(secret)?.try_into().ok()?;
        Some(Self { key, skeleton })
    }
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

/// What a client's `nonce` is hashed with, so that the answer cannot be
/// mistaken for the answer to the `hello` challenge.
const SERVER_PROOF_PREFIX: &[u8] = b"aurora-hello:";

/// The `nonce` of an `auth` message is 16 to 64 bytes, in hex.
const NONCE_BYTES: std::ops::RangeInclusive<usize> = 16..=64;

/// The server's answer to a client nonce (hex): HMAC-SHA256(key,
/// "aurora-hello:" followed by the nonce bytes). `None` when the nonce is not
/// valid hex or has a wrong length.
pub(super) fn server_proof(key: &[u8; 32], nonce: &str) -> Option<String> {
    let nonce = unhex(nonce).filter(|bytes| NONCE_BYTES.contains(&bytes.len()))?;
    let mut message = SERVER_PROOF_PREFIX.to_vec();
    message.extend_from_slice(&nonce);
    Some(hex(&hmac_sha256(key, &message)))
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
        /// A client nonce (hex, checked by `server_proof`) asking for the
        /// server's proof in the answer.
        nonce: Option<String>,
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
        nonce: Option<String>,
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
            Some(response) => match raw.nonce {
                Some(nonce) if server_proof(&[0; 32], &nonce).is_none() => Err("auth nonce is not 16 to 64 bytes in hex".into()),
                nonce => Ok(Message::Auth {
                    client: raw.client.map(|name| name.chars().filter(|c| !c.is_control()).take(64).collect()),
                    response,
                    nonce,
                }),
            },
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
