use std::{env, error::Error, io::{self, Read, Write}, time::Duration};

// A duplicate or out-of-sync ping is answered with the existing count.
fn accept(current: u32, ping: u32) -> u32 {
    if current == ping { current.wrapping_add(1) } else { current }
}
fn parse_ping(line: &[u8]) -> Option<u32> {
    let s = std::str::from_utf8(line).ok()?.trim();
    let n = s.strip_prefix("PING ")?;
    if n.is_empty() || !n.bytes().all(|b| b.is_ascii_digit()) { return None; }
    n.parse().ok()
}
fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = env::args().collect();
    if args.len() != 2 {
        eprintln!("Usage: {} <serial-port>\nExample: {} /dev/ttyACM0", args[0], args[0]);
        std::process::exit(2);
    }
    let mut port = serialport::new(&args[1], 115_200)
        .timeout(Duration::from_millis(200)).open()?;
    println!("Listening on {} at 115200 8N1; Ctrl-C to quit", args[1]);
    let mut accepted = 0u32;
    let mut line = Vec::with_capacity(48);
    let mut overflow = false;
    let mut buf = [0u8; 128];
    loop {
        match port.read(&mut buf) {
            Ok(n) => for &b in &buf[..n] {
                if b == b'\n' {
                    if !overflow {
                        if let Some(ping) = parse_ping(&line) {
                            let previous = accepted;
                            accepted = accept(accepted, ping);
                            println!("PING {ping} -> PONG {accepted}{}", if ping != previous { " (resync/retry)" } else { "" });
                            write!(port, "PONG {accepted}\n")?;
                            port.flush()?;
                        } else if !line.is_empty() { eprintln!("Ignoring malformed line: {:?}", String::from_utf8_lossy(&line)); }
                    }
                    line.clear(); overflow = false;
                } else if line.len() < 48 && !overflow { line.push(b); }
                else { overflow = true; }
            },
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {},
            Err(e) => return Err(e.into()),
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test] fn retry_and_resync() {
        assert_eq!(accept(0, 0), 1);
        assert_eq!(accept(1, 0), 1); // lost pong, no double increment
        assert_eq!(accept(1, 1), 2);
        assert_eq!(accept(20, 7), 20); // report host's count
        assert_eq!(accept(u32::MAX, u32::MAX), 0);
    }
    #[test] fn framing() {
        assert_eq!(parse_ping(b"PING 12\r"), Some(12));
        assert_eq!(parse_ping(b"PING -1"), None);
        assert_eq!(parse_ping(b"PING 4294967296"), None);
    }
}
