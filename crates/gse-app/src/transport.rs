//! Byte transports: USB CDC serial port or a TCP socket (simulator / remote).
//!
//! The protocol engine only sees [`ByteLink`], so the simulator can exercise
//! the exact same framing, retry and safety code paths as real hardware.

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};

use crate::serial::SerialPort;

pub trait ByteLink: Send {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    fn write_all(&mut self, data: &[u8]) -> io::Result<()>;
    fn name(&self) -> String;
    fn is_tcp(&self) -> bool {
        false
    }
}

pub struct SerialLink {
    port: SerialPort,
}

impl SerialLink {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        Ok(SerialLink {
            port: SerialPort::open(path, baud)?,
        })
    }
}

impl ByteLink for SerialLink {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.port.read(buf)
    }

    fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        self.port.write_all(data)
    }

    fn name(&self) -> String {
        self.port.path().to_string()
    }
}

pub struct TcpLink {
    stream: TcpStream,
    name: String,
}

impl TcpLink {
    pub fn connect(spec: &str) -> io::Result<Self> {
        let addr = strip_tcp_prefix(spec);
        let mut last_err = None;
        for sock in addr.to_socket_addrs()? {
            match TcpStream::connect_timeout(&sock, Duration::from_millis(800)) {
                Ok(stream) => {
                    stream.set_nodelay(true).ok();
                    stream.set_nonblocking(true)?;
                    return Ok(TcpLink {
                        stream,
                        name: format!("tcp://{addr}"),
                    });
                }
                Err(e) => last_err = Some(e),
            }
        }
        Err(last_err.unwrap_or_else(|| {
            io::Error::new(io::ErrorKind::AddrNotAvailable, format!("cannot resolve {addr}"))
        }))
    }
}

impl ByteLink for TcpLink {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.stream.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(e) if e.kind() == io::ErrorKind::Interrupted => Ok(0),
            Err(e) => Err(e),
        }
    }

    fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut written = 0usize;
        while written < data.len() {
            match self.stream.write(&data[written..]) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::WriteZero,
                        "TCP connection closed",
                    ))
                }
                Ok(n) => written += n,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                    if Instant::now() >= deadline {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "TCP write timed out",
                        ));
                    }
                    std::thread::sleep(Duration::from_millis(1));
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn is_tcp(&self) -> bool {
        true
    }
}

pub fn strip_tcp_prefix(spec: &str) -> &str {
    spec.strip_prefix("tcp://")
        .or_else(|| spec.strip_prefix("tcp:"))
        .unwrap_or(spec)
}

pub fn is_tcp_spec(spec: &str) -> bool {
    spec.starts_with("tcp://") || spec.starts_with("tcp:")
}

/// Open a link for a user-specified port string ("auto" is handled by the
/// engine, which probes every enumerated port).
pub fn open_link(spec: &str, baud: u32) -> io::Result<Box<dyn ByteLink>> {
    if is_tcp_spec(spec) {
        Ok(Box::new(TcpLink::connect(spec)?))
    } else {
        Ok(Box::new(SerialLink::open(spec, baud)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tcp_prefixes() {
        assert_eq!(strip_tcp_prefix("tcp://127.0.0.1:9000"), "127.0.0.1:9000");
        assert_eq!(strip_tcp_prefix("tcp:127.0.0.1:9000"), "127.0.0.1:9000");
        assert_eq!(strip_tcp_prefix("/dev/ttyACM0"), "/dev/ttyACM0");
        assert!(is_tcp_spec("tcp://localhost:1"));
        assert!(!is_tcp_spec("COM7"));
    }

    #[test]
    fn missing_port_is_an_error() {
        let result = SerialLink::open("/dev/definitely-not-a-port-gse", 115_200);
        assert!(result.is_err());
    }
}
