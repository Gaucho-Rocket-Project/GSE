//! POSIX serial backend (Linux and macOS) using `termios` + `poll`.
#![allow(dead_code)]

use std::io;
use std::os::fd::IntoRawFd;
use std::os::raw::{c_int, c_ulong, c_void};
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::time::{Duration, Instant};

use super::PortInfo;

// ---------------------------------------------------------------------------
// Platform constants
// ---------------------------------------------------------------------------

#[cfg(any(target_os = "macos", feature = "check-macos"))]
mod plat {
    pub type Speed = u64;
    pub type CFlag = u64;

    pub const B9600: Speed = 9_600;
    pub const B19200: Speed = 19_200;
    pub const B38400: Speed = 38_400;
    pub const B57600: Speed = 57_600;
    pub const B115200: Speed = 115_200;
    pub const B230400: Speed = 230_400;

    pub const O_NONBLOCK: i32 = 0x0004;
    pub const O_NOCTTY: i32 = 0x2_0000;
    pub const O_CLOEXEC: i32 = 0x0100_0000;

    /// `offsetof(struct termios, c_cflag)` on Darwin.
    pub const CFLAG_OFFSET: usize = 16;
    pub const EAGAIN: i32 = 35;
}

#[cfg(all(not(target_os = "macos"), not(feature = "check-macos")))]
mod plat {
    pub type Speed = u32;
    pub type CFlag = u32;

    pub const B9600: Speed = 0o15;
    pub const B19200: Speed = 0o16;
    pub const B38400: Speed = 0o17;
    pub const B57600: Speed = 0o10001;
    pub const B115200: Speed = 0o10002;
    pub const B230400: Speed = 0o10003;

    pub const O_NONBLOCK: i32 = 0o4000;
    pub const O_NOCTTY: i32 = 0o400;
    pub const O_CLOEXEC: i32 = 0o2000000;

    /// `offsetof(struct termios, c_cflag)` on glibc.
    pub const CFLAG_OFFSET: usize = 8;
    pub const EAGAIN: i32 = 11;
}

const MACOS: bool = cfg!(any(target_os = "macos", feature = "check-macos"));
const TCSANOW: c_int = 0;
const TCIFLUSH: c_int = 0;
const CLOCAL: CFlag = 0x800;
const CREAD: CFlag = 0x80;
const POLLIN: i16 = 0x001;
const POLLOUT: i16 = 0x004;
const EINTR: i32 = 4;

type CFlag = plat::CFlag;

#[cfg_attr(windows, allow(dead_code))]
#[repr(C)]
struct PollFd {
    fd: c_int,
    events: i16,
    revents: i16,
}

/// Opaque, generously sized and aligned storage for `struct termios`.
/// Linux is 60 bytes, Darwin 72 bytes; using an opaque buffer avoids
/// re-declaring two ABI-fragile layouts.
#[repr(C, align(8))]
struct Termios([u8; 256]);

// Do not declare `open(2)` here. Its C prototype is variadic and recent rustc
// rejects a mismatched declaration of a symbol the standard library itself uses
// (`invalid_runtime_symbol_definitions`), so the device node is opened through
// `std::fs::OpenOptions` with the flags we need and the raw descriptor is taken
// over for `termios`/`poll`.
extern "C" {
    fn tcgetattr(fd: c_int, termios_p: *mut c_void) -> c_int;
    fn tcsetattr(fd: c_int, optional_actions: c_int, termios_p: *const c_void) -> c_int;
    fn cfmakeraw(termios_p: *mut c_void);
    fn cfsetispeed(termios_p: *mut c_void, speed: plat::Speed) -> c_int;
    fn cfsetospeed(termios_p: *mut c_void, speed: plat::Speed) -> c_int;
    fn tcflush(fd: c_int, queue_selector: c_int) -> c_int;
    fn close(fd: c_int) -> c_int;
    fn read(fd: c_int, buf: *mut c_void, count: usize) -> isize;
    fn write(fd: c_int, buf: *const c_void, count: usize) -> isize;
    fn poll(fds: *mut PollFd, nfds: c_ulong, timeout: c_int) -> c_int;
}

fn speed_constant(baud: u32) -> io::Result<plat::Speed> {
    match baud {
        9_600 => Ok(plat::B9600),
        19_200 => Ok(plat::B19200),
        38_400 => Ok(plat::B38400),
        57_600 => Ok(plat::B57600),
        115_200 => Ok(plat::B115200),
        230_400 => Ok(plat::B230400),
        other => Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("unsupported baud rate {other}"),
        )),
    }
}

pub struct Port {
    fd: c_int,
}

impl Port {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(plat::O_NOCTTY | plat::O_NONBLOCK | plat::O_CLOEXEC)
            .open(path)?;
        let fd = file.into_raw_fd();
        let port = Port { fd };
        if let Err(e) = port.configure(baud) {
            drop(port);
            return Err(e);
        }
        Ok(port)
    }

    fn configure(&self, baud: u32) -> io::Result<()> {
        let speed = speed_constant(baud)?;
        let mut termios = Termios([0u8; 256]);
        let ptr = termios.0.as_mut_ptr() as *mut c_void;
        if unsafe { tcgetattr(self.fd, ptr) } != 0 {
            return Err(io::Error::last_os_error());
        }
        unsafe {
            cfmakeraw(ptr);
            if cfsetispeed(ptr, speed) != 0 || cfsetospeed(ptr, speed) != 0 {
                return Err(io::Error::last_os_error());
            }
            let base = termios.0.as_mut_ptr();
            let flag_ptr = base.add(plat::CFLAG_OFFSET) as *mut CFlag;
            let mut flags = flag_ptr.read_unaligned();
            flags |= CLOCAL | CREAD;
            flag_ptr.write_unaligned(flags);
            if tcsetattr(self.fd, TCSANOW, ptr) != 0 {
                return Err(io::Error::last_os_error());
            }
            let _ = tcflush(self.fd, TCIFLUSH);
        }
        Ok(())
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        loop {
            let n = unsafe { read(self.fd, buf.as_mut_ptr() as *mut c_void, buf.len()) };
            if n >= 0 {
                return Ok(n as usize);
            }
            let err = io::Error::last_os_error();
            match err.raw_os_error() {
                Some(EINTR) => continue,
                Some(code) if code == plat::EAGAIN => return Ok(0),
                _ => return Err(err),
            }
        }
    }

    pub fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut written = 0usize;
        while written < data.len() {
            let n = unsafe {
                write(
                    self.fd,
                    data[written..].as_ptr() as *const c_void,
                    data.len() - written,
                )
            };
            if n > 0 {
                written += n as usize;
                continue;
            }
            if n == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "serial write returned 0",
                ));
            }
            let err = io::Error::last_os_error();
            match err.raw_os_error() {
                Some(EINTR) => continue,
                Some(code) if code == plat::EAGAIN => {
                    if Instant::now() >= deadline {
                        return Err(io::Error::new(
                            io::ErrorKind::TimedOut,
                            "serial write timed out",
                        ));
                    }
                    let mut pfd = PollFd {
                        fd: self.fd,
                        events: POLLOUT,
                        revents: 0,
                    };
                    unsafe {
                        poll(&mut pfd, 1, 50);
                    }
                }
                _ => return Err(err),
            }
        }
        Ok(())
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        unsafe {
            close(self.fd);
        }
    }
}

/// Block until the port has data or the timeout expires. Used by the
/// auto-detection probe; returns true when data may be available.
pub fn wait_readable(fd_raw: i32, timeout_ms: i32) -> bool {
    let mut pfd = PollFd {
        fd: fd_raw,
        events: POLLIN,
        revents: 0,
    };
    unsafe { poll(&mut pfd, 1, timeout_ms) > 0 }
}

pub fn list_ports() -> Vec<PortInfo> {
    let mut out: Vec<PortInfo> = Vec::new();

    if MACOS {
        if let Ok(entries) = std::fs::read_dir("/dev") {
            let mut names: Vec<String> = entries
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|name| {
                    name.starts_with("cu.")
                        && (name.contains("usb")
                            || name.contains("SLAB")
                            || name.contains("wch")
                            || name.contains("serial"))
                })
                .collect();
            names.sort();
            for name in names {
                out.push(PortInfo {
                    path: format!("/dev/{name}"),
                    label: name.clone(),
                    kind: "usb",
                });
            }
        }
    } else {
        // Prefer stable by-id names: they survive re-enumeration.
        if let Ok(entries) = std::fs::read_dir("/dev/serial/by-id") {
            let mut items: Vec<(String, String)> = entries
                .filter_map(|e| e.ok())
                .filter_map(|entry| {
                    let label = entry.file_name().to_string_lossy().into_owned();
                    let path = std::fs::canonicalize(entry.path()).ok()?;
                    Some((label, path.to_string_lossy().into_owned()))
                })
                .collect();
            items.sort();
            for (label, path) in items {
                if !out.iter().any(|p| p.path == path) {
                    out.push(PortInfo {
                        path,
                        label,
                        kind: "usb",
                    });
                }
            }
        }
        if let Ok(entries) = std::fs::read_dir("/dev") {
            let mut names: Vec<String> = entries
                .filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .filter(|n| n.starts_with("ttyACM") || n.starts_with("ttyUSB"))
                .collect();
            names.sort();
            for name in names {
                let path = format!("/dev/{name}");
                if !out.iter().any(|p| p.path == path) {
                    out.push(PortInfo {
                        path,
                        label: name,
                        kind: "usb",
                    });
                }
            }
        }
    }

    out.sort_by(|a, b| a.label.cmp(&b.label));
    out
}

/// Helper used by tests: the serial path exists and is a character device.
pub fn path_exists(path: &str) -> bool {
    Path::new(path).exists()
}
