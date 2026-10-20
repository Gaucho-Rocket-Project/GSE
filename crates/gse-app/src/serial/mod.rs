//! Cross-platform serial port access.
//!
//! Implemented with direct OS APIs so the app has **zero crate dependencies**
//! and can be built offline on all three supported operating systems:
//!
//! * Linux / macOS: `termios` + `poll(2)` on a descriptor opened by `std::fs`
//! * Windows: `CreateFileA` + `SetCommState` + `ReadFile` / `WriteFile`
//!
//! The NUCLEO-H753ZI ground board exposes its ST-LINK virtual COM port
//! (USART3, PD8/PD9) as a standard CDC-ACM device, so no vendor driver is
//! required on Windows 10+, Linux (cdc_acm) or macOS.

use std::io;

#[cfg(unix)]
mod unix;
#[cfg(any(windows, feature = "check-windows"))]
mod windows;

#[cfg(all(unix, not(feature = "check-windows")))]
use self::unix as imp;
#[cfg(any(windows, feature = "check-windows"))]
use self::windows as imp;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PortInfo {
    pub path: String,
    pub label: String,
    /// `usb` for ports that look like a USB CDC adapter, `other` otherwise.
    pub kind: &'static str,
}

pub struct SerialPort {
    inner: imp::Port,
}

impl SerialPort {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        Ok(SerialPort {
            inner: imp::Port::open(path, baud)?,
        })
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.inner.read(buf)
    }

    pub fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        self.inner.write_all(data)
    }
}

/// Enumerate candidate serial ports. Order is stable (by label).
pub fn list_ports() -> Vec<PortInfo> {
    imp::list_ports()
}

/// True if the path looks like a serial device path for the current OS.
///
/// Used by the platform scanners and kept public for callers that accept a
/// user-supplied path.
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
        for p in ports {
            assert!(!p.path.is_empty());
        }
    }
}
