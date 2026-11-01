//! Deliberately minimal host responder copied from the `pingpong` branch.
//! This is only used by `gse-app --pingpong-test` when the feature is enabled.

use std::{
    io::{self, Read, Write},
    time::Duration,
};

fn accept(current: u32, ping: u32) -> u32 {
    if current == ping {
        current.wrapping_add(1)
    } else {
        current
    }
}

fn parse_ping(line: &[u8]) -> Option<u32> {
    let s = std::str::from_utf8(line).ok()?.trim();
    let n = s.strip_prefix("PING ")?;
    if n.is_empty() || !n.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    n.parse().ok()
}

pub fn run(port_name: &str) -> Result<(), String> {
    let mut port = serialport::new(port_name, 115_200)
        .timeout(Duration::from_millis(200))
        .open()
        .map_err(|e| format!("cannot open {port_name} at 115200 baud: {e}"))?;
    println!("PING/PONG regression listening on {port_name} at 115200 8N1; Ctrl-C to quit");

    let mut accepted = 0u32;
    let mut line = Vec::with_capacity(48);
    let mut overflow = false;
    let mut buf = [0u8; 128];
    loop {
        match port.read(&mut buf) {
            Ok(n) => {
                for &byte in &buf[..n] {
                    if byte == b'\n' {
                        if !overflow {
                            if let Some(ping) = parse_ping(&line) {
                                let previous = accepted;
                                accepted = accept(accepted, ping);
                                println!(
                                    "PING {ping} -> PONG {accepted}{}",
                                    if ping != previous { " (resync/retry)" } else { "" }
                                );
                                write!(port, "PONG {accepted}\n")
                                    .map_err(|e| format!("serial write failed: {e}"))?;
                                port.flush()
                                    .map_err(|e| format!("serial flush failed: {e}"))?;
                            } else if !line.is_empty() {
                                eprintln!(
                                    "Ignoring malformed line: {:?}",
                                    String::from_utf8_lossy(&line)
                                );
                            }
                        }
                        line.clear();
                        overflow = false;
                    } else if line.len() < 48 && !overflow {
                        line.push(byte);
                    } else {
                        overflow = true;
                    }
                }
            }
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {}
            Err(e) => return Err(format!("serial read failed: {e}")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retry_and_resync_are_idempotent() {
        assert_eq!(accept(0, 0), 1);
        assert_eq!(accept(1, 0), 1); // lost PONG: do not increment twice
        assert_eq!(accept(1, 1), 2);
        assert_eq!(accept(20, 7), 20); // report host's count after reset
        assert_eq!(accept(u32::MAX, u32::MAX), 0);
    }

    #[test]
    fn line_framing_rejects_invalid_counters() {
        assert_eq!(parse_ping(b"PING 12\r"), Some(12));
        assert_eq!(parse_ping(b"PING -1"), None);
        assert_eq!(parse_ping(b"PING 4294967296"), None);
    }
}
