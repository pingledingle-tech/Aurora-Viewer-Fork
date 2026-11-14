//! Opt-in loopback link with the Pingmator animation editor (Aurora addition,
//! no Legacy Viewer counterpart). Pingmator sends its current pose as a small
//! BVH; the viewer shows it on the user's own avatar through the local
//! poser, visible to the user only. Nothing reaches the simulator.
//!
//! Protocol (version 1): one TCP client on 127.0.0.1, newline-delimited JSON
//! in both directions. The viewer greets with
//! `{"type":"hello","app":"Aurora","protocol":1,"features":["pose"]}`; the
//! client sends `{"type":"pose","bvh":"…"}` whenever its pose changes, and
//! may send `{"type":"hello",…}` and empty heartbeat lines. The viewer
//! answers a pose it cannot read with `{"type":"error","message":"…"}`.
//! The socket, the parsing and the BVH decoding live on a dedicated thread;
//! the frame only takes the latest decoded pose.

use aurora_assets::{Animation, Skeleton};
use parking_lot::Mutex;
use std::{
    io::{self, BufRead, BufReader, Write},
    net::{Ipv4Addr, TcpListener, TcpStream},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

pub const DEFAULT_PORT: u16 = 17656;
pub const PROTOCOL: u32 = 1;
/// A Bento pose is about 20 KB of BVH; anything far above is not a pose.
const MAX_LINE: usize = 4 * 1024 * 1024;

#[derive(Default)]
struct Shared {
    stop: AtomicBool,
    /// Set once per connection, taken by the frame for the chat notice.
    connected_notice: AtomicBool,
    client: Mutex<Option<String>>,
    latest: Mutex<Option<Animation>>,
    error: Mutex<Option<String>>,
}

/// The link's state in the app: off until the user turns it on, never saved.
#[derive(Default)]
pub struct Link {
    shared: Option<Arc<Shared>>,
    /// A decoded pose waiting for the poser to have a session for the agent.
    pending: Option<Animation>,
}

impl Link {
    pub fn enabled(&self) -> bool {
        self.shared.is_some()
    }
    /// Start listening. Fails when the port is taken, or before the avatar
    /// skeleton is loaded (the BVH joints are matched against it).
    pub fn start(&mut self, port: u16, skeleton: Option<&Skeleton>) -> Result<(), String> {
        self.stop();
        let skeleton = skeleton.cloned().ok_or_else(|| aurora_i18n::tr!("pingmator-link-no-skeleton").to_string())?;
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, port)).map_err(|e| e.to_string())?;
        self.shared = Some(start_listener(listener, skeleton).map_err(|e| e.to_string())?);
        log::info!("pingmator link: listening on 127.0.0.1:{port}");
        Ok(())
    }
    pub fn stop(&mut self) {
        if let Some(shared) = self.shared.take() {
            shared.stop.store(true, Ordering::Release);
            log::info!("pingmator link: stopped");
        }
        self.pending = None;
    }
    /// The newest pose, keeping it until [`Link::keep`] gives it back unused.
    pub fn take_pose(&mut self) -> Option<Animation> {
        let latest = self.shared.as_ref().and_then(|s| s.latest.lock().take());
        latest.or_else(|| self.pending.take())
    }
    /// A pose the poser could not take yet (no session for the agent).
    pub fn keep(&mut self, animation: Animation) {
        self.pending = Some(animation);
    }
    /// The client's name once per connection, for a chat notice.
    pub fn take_connected(&mut self) -> Option<String> {
        let shared = self.shared.as_ref()?;
        shared
            .connected_notice
            .swap(false, Ordering::AcqRel)
            .then(|| shared.client.lock().clone().unwrap_or_else(|| "Pingmator".into()))
    }
    pub fn take_error(&mut self) -> Option<String> {
        self.shared.as_ref().and_then(|s| s.error.lock().take())
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        self.stop();
    }
}

fn start_listener(listener: TcpListener, skeleton: Skeleton) -> io::Result<Arc<Shared>> {
    listener.set_nonblocking(true)?;
    let shared = Arc::new(Shared::default());
    let worker = shared.clone();
    std::thread::Builder::new().name("pingmator-link".into()).spawn(move || {
        while !worker.stop.load(Ordering::Acquire) {
            match listener.accept() {
                Ok((stream, _)) => {
                    worker.connected_notice.store(true, Ordering::Release);
                    if let Err(error) = serve(stream, &worker, &skeleton)
                        && !matches!(
                            error.kind(),
                            io::ErrorKind::UnexpectedEof
                                | io::ErrorKind::ConnectionReset
                                | io::ErrorKind::ConnectionAborted
                                | io::ErrorKind::Interrupted
                        )
                    {
                        log::warn!("pingmator link: {error}");
                    }
                    worker.connected_notice.store(false, Ordering::Release);
                    worker.client.lock().take();
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => std::thread::sleep(Duration::from_millis(50)),
                Err(e) => {
                    log::warn!("pingmator link: accept failed: {e}");
                    *worker.error.lock() = Some(e.to_string());
                    break;
                }
            }
        }
    })?;
    Ok(shared)
}

/// One message of the client, already split from its line.
#[derive(Debug, PartialEq)]
enum Message {
    Hello(Option<String>),
    Pose(String),
    Heartbeat,
    Unknown,
}

fn parse(line: &[u8]) -> Result<Message, String> {
    if line.iter().all(u8::is_ascii_whitespace) {
        return Ok(Message::Heartbeat);
    }
    let value: serde_json::Value = serde_json::from_slice(line).map_err(|e| e.to_string())?;
    Ok(match value.get("type").and_then(|t| t.as_str()) {
        Some("hello") => Message::Hello(value.get("app").and_then(|a| a.as_str()).map(|a| a.chars().take(64).collect())),
        Some("pose") => Message::Pose(
            value
                .get("bvh")
                .and_then(|b| b.as_str())
                .ok_or_else(|| "pose without bvh".to_string())?
                .to_owned(),
        ),
        _ => Message::Unknown,
    })
}

/// The pose a BVH holds: its first frame, as the poser's Load reads it.
fn decode(bvh: &str, skeleton: &Skeleton) -> Result<Animation, String> {
    aurora_assets::import_animation::parse_bvh_pose(bvh.as_bytes(), skeleton)
        .map(|b| b.animation)
        .map_err(|e| e.to_string())
}

fn send(stream: &mut TcpStream, value: serde_json::Value) -> io::Result<()> {
    writeln!(stream, "{value}")?;
    stream.flush()
}

fn serve(stream: TcpStream, shared: &Shared, skeleton: &Skeleton) -> io::Result<()> {
    stream.set_nonblocking(false)?;
    stream.set_read_timeout(Some(Duration::from_millis(200)))?;
    stream.set_write_timeout(Some(Duration::from_secs(2)))?;
    stream.set_nodelay(true)?;
    let mut writer = stream.try_clone()?;
    send(
        &mut writer,
        serde_json::json!({ "type": "hello", "app": "Aurora", "protocol": PROTOCOL, "features": ["pose"] }),
    )?;
    let mut reader = BufReader::new(stream);
    let mut line = Vec::new();
    while !shared.stop.load(Ordering::Acquire) {
        match reader.read_until(b'\n', &mut line) {
            Ok(0) => return Err(io::ErrorKind::UnexpectedEof.into()),
            Ok(_) if !line.ends_with(b"\n") => {}
            Ok(_) => {
                match parse(&line) {
                    Ok(Message::Pose(bvh)) => match decode(&bvh, skeleton) {
                        Ok(animation) => *shared.latest.lock() = Some(animation),
                        Err(message) => send(&mut writer, serde_json::json!({ "type": "error", "message": message }))?,
                    },
                    Ok(Message::Hello(app)) => *shared.client.lock() = app,
                    Ok(Message::Heartbeat | Message::Unknown) => {}
                    Err(message) => send(&mut writer, serde_json::json!({ "type": "error", "message": message }))?,
                }
                line.clear();
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut) => {}
            Err(e) => return Err(e),
        }
        if line.len() > MAX_LINE {
            send(&mut writer, serde_json::json!({ "type": "error", "message": "message too large" }))?;
            return Err(io::ErrorKind::InvalidData.into());
        }
    }
    Err(io::ErrorKind::Interrupted.into())
}

#[cfg(test)]
mod tests;
