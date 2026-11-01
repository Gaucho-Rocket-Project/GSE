//! Cross-platform serial access shared by the normal GSE transport and the
//! opt-in PING/PONG regression responder.
//!
//! The app deliberately uses the same `serialport` backend in both paths so
//! their device opening, line configuration, and OS-specific I/O behavior do
//! not drift. The transport engine treats reads as nonblocking: an empty read
//! is `Ok(0)`, not a timeout error.

use std::io::{self, Read, Write};
use std::time::Duration;

const READ_TIMEOUT: Duration = Duration::from_millis(5);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortInfo {
    pub path: String,
    pub label: String,
    /// `usb` for USB serial devices, `other` otherwise.
    pub kind: &'static str,
}

pub struct SerialPort {
    inner: Box<dyn serialport::SerialPort>,
}

impl SerialPort {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let inner = serialport::new(path, baud)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(serial_error)?;
        Ok(Self { inner })
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        nonblocking_read(self.inner.read(buf))
    }

    pub fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        self.inner.write_all(data)
    }

    pub fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.inner.set_baud_rate(baud).map_err(serial_error)
    }
}

fn serial_error(error: serialport::Error) -> io::Error {
    io::Error::other(error)
}

/// Preserve the `ByteLink` contract used by the engine: no available bytes is
/// not a disconnect. `serialport` reports this condition as a timed-out read.
fn nonblocking_read(result: io::Result<usize>) -> io::Result<usize> {
    match result {
        Err(error)
            if matches!(
                error.kind(),
                io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted
            ) =>
        {
            Ok(0)
        }
        other => other,
    }
}

/// Enumerate candidate serial ports in a stable label order.
pub fn list_ports() -> Vec<PortInfo> {
    let mut ports: Vec<_> = serialport::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|info| {
            let kind = match info.port_type {
                serialport::SerialPortType::UsbPort(_) => "usb",
                _ => "other",
            };
            PortInfo {
                path: info.port_name.clone(),
                label: info.port_name,
                kind,
            }
        })
        .collect();
    ports.sort_by_key(|port| port.label.clone());
    ports
}

/// True if the path looks like a serial device path for the current OS.
#[cfg_attr(not(test), allow(dead_code))]
pub fn looks_like_serial_path(path: &str) -> bool {
    if path.starts_with("tcp://") || path.starts_with("tcp:") {
        return false;
    }
    #[cfg(windows)]
    {
        path.len() > 3 && path[..3].eq_ignore_ascii_case("COM")
    }
    #[cfg(unix)]
    {
        path.starts_with("/dev/")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_serial_reads_are_nonblocking() {
        assert_eq!(
            nonblocking_read(Err(io::Error::from(io::ErrorKind::TimedOut))).unwrap(),
            0
        );
        assert_eq!(
            nonblocking_read(Err(io::Error::from(io::ErrorKind::WouldBlock))).unwrap(),
            0
        );
        assert_eq!(
            nonblocking_read(Err(io::Error::from(io::ErrorKind::Interrupted))).unwrap(),
            0
        );
    }

    #[test]
    fn real_serial_errors_are_preserved() {
        assert_eq!(
            nonblocking_read(Err(io::Error::from(io::ErrorKind::BrokenPipe)))
                .unwrap_err()
                .kind(),
            io::ErrorKind::BrokenPipe
        );
    }

    #[test]
    fn tcp_is_not_serial() {
        assert!(!looks_like_serial_path("tcp://127.0.0.1:7000"));
    }

    #[cfg(unix)]
    #[test]
    fn dev_paths_are_serial() {
        assert!(looks_like_serial_path("/dev/ttyACM0"));
    }

    #[test]
    fn listing_does_not_panic() {
        let ports = list_ports();
        for port in ports {
            assert!(!port.path.is_empty());
        }
    }
}
