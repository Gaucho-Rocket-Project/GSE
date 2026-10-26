//! Windows serial backend using the Win32 comm API.
//!
//! Compiled on Windows and, for type-checking only, on other hosts with the
//! `check-windows` feature (`cargo check --features check-windows`).
#![allow(dead_code)]

use std::ffi::CString;
use std::io;
use std::os::raw::{c_char, c_int, c_void};
use std::time::{Duration, Instant};

use super::PortInfo;

type Handle = *mut c_void;
type Hkey = *mut c_void;

const INVALID_HANDLE_VALUE: Handle = -1isize as Handle;
const GENERIC_READ: u32 = 0x8000_0000;
const GENERIC_WRITE: u32 = 0x4000_0000;
const OPEN_EXISTING: u32 = 3;
const KEY_READ: u32 = 0x0002_0019;
const ERROR_SUCCESS: u32 = 0;
const ERROR_NO_MORE_ITEMS: u32 = 259;
const PURGE_RXABORT: u32 = 0x0002;
const PURGE_RXCLEAR: u32 = 0x0008;
const PURGE_TXABORT: u32 = 0x0001;
const PURGE_TXCLEAR: u32 = 0x0004;

fn hkey_local_machine() -> Hkey {
    0x8000_0002u32 as usize as Hkey
}

#[repr(C)]
#[derive(Default)]
struct Dcb {
    dcblength: u32,
    baudrate: u32,
    flags: u32,
    w_reserved: u16,
    xon_lim: u16,
    xoff_lim: u16,
    byte_size: u8,
    parity: u8,
    stop_bits: u8,
    xon_char: i8,
    xoff_char: i8,
    error_char: i8,
    eof_char: i8,
    evt_char: i8,
    w_reserved1: u16,
}

#[repr(C)]
struct CommTimeouts {
    read_interval_timeout: u32,
    read_total_timeout_multiplier: u32,
    read_total_timeout_constant: u32,
    write_total_timeout_multiplier: u32,
    write_total_timeout_constant: u32,
}

#[link(name = "kernel32")]
extern "system" {
    fn CreateFileA(
        file_name: *const c_char,
        desired_access: u32,
        share_mode: u32,
        security_attributes: *mut c_void,
        creation_disposition: u32,
        flags_and_attributes: u32,
        template_file: Handle,
    ) -> Handle;
    fn CloseHandle(handle: Handle) -> c_int;
    fn GetCommState(handle: Handle, dcb: *mut Dcb) -> c_int;
    fn SetCommState(handle: Handle, dcb: *const Dcb) -> c_int;
    fn SetCommTimeouts(handle: Handle, timeouts: *const CommTimeouts) -> c_int;
    fn PurgeComm(handle: Handle, flags: u32) -> c_int;
    fn ReadFile(
        handle: Handle,
        buffer: *mut u8,
        bytes_to_read: u32,
        bytes_read: *mut u32,
        overlapped: *mut c_void,
    ) -> c_int;
    fn WriteFile(
        handle: Handle,
        buffer: *const u8,
        bytes_to_write: u32,
        bytes_written: *mut u32,
        overlapped: *mut c_void,
    ) -> c_int;
    fn GetLastError() -> u32;
}

#[link(name = "advapi32")]
extern "system" {
    fn RegOpenKeyExA(
        key: Hkey,
        sub_key: *const c_char,
        options: u32,
        sam_desired: u32,
        result: *mut Hkey,
    ) -> u32;
    fn RegEnumValueA(
        key: Hkey,
        index: u32,
        value_name: *mut c_char,
        value_name_len: *mut u32,
        reserved: *mut u32,
        value_type: *mut u32,
        data: *mut u8,
        data_len: *mut u32,
    ) -> u32;
    fn RegCloseKey(key: Hkey) -> u32;
}

pub struct Port {
    handle: Handle,
}

// A Win32 `HANDLE` is a plain value and the comm API allows reads and writes
// from any single thread at a time. `Port` owns the handle (closed in `Drop`)
// and is only ever used from one thread, so moving it across threads is sound.
unsafe impl Send for Port {}

impl Port {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let full = if path.starts_with("\\\\.\\") {
            path.to_string()
        } else {
            format!("\\\\.\\{path}")
        };
        let name = CString::new(full)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "invalid COM port name"))?;
        let handle = unsafe {
            CreateFileA(
                name.as_ptr(),
                GENERIC_READ | GENERIC_WRITE,
                0,
                std::ptr::null_mut(),
                OPEN_EXISTING,
                0,
                std::ptr::null_mut(),
            )
        };
        if handle == INVALID_HANDLE_VALUE {
            return Err(io::Error::last_os_error());
        }
        let port = Port { handle };
        port.configure(baud)?;
        Ok(port)
    }

    pub fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.configure(baud)
    }

    fn configure(&self, baud: u32) -> io::Result<()> {
        let mut dcb = Dcb {
            dcblength: std::mem::size_of::<Dcb>() as u32,
            baudrate: baud,
            // fBinary, DTR_CONTROL_ENABLE, TX_CONTINUE_ON_XOFF, RTS_CONTROL_ENABLE
            flags: 1 | (1 << 4) | (1 << 7) | (1 << 12),
            byte_size: 8,
            parity: 0,    // NOPARITY
            stop_bits: 0, // ONESTOPBIT
            ..Default::default()
        };
        unsafe {
            if GetCommState(self.handle, &mut dcb) == 0 {
                return Err(io::Error::last_os_error());
            }
            dcb.dcblength = std::mem::size_of::<Dcb>() as u32;
            dcb.baudrate = baud;
            dcb.byte_size = 8;
            dcb.parity = 0;
            dcb.stop_bits = 0;
            dcb.flags = 1 | (1 << 4) | (1 << 7) | (1 << 12);
            if SetCommState(self.handle, &dcb) == 0 {
                return Err(io::Error::last_os_error());
            }
            let timeouts = CommTimeouts {
                // Return immediately with whatever is available.
                read_interval_timeout: u32::MAX,
                read_total_timeout_multiplier: 0,
                read_total_timeout_constant: 0,
                write_total_timeout_multiplier: 0,
                write_total_timeout_constant: 2_000,
            };
            if SetCommTimeouts(self.handle, &timeouts) == 0 {
                return Err(io::Error::last_os_error());
            }
            PurgeComm(
                self.handle,
                PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR,
            );
        }
        Ok(())
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mut read = 0u32;
        let ok = unsafe {
            ReadFile(
                self.handle,
                buf.as_mut_ptr(),
                buf.len().min(u32::MAX as usize) as u32,
                &mut read,
                std::ptr::null_mut(),
            )
        };
        if ok == 0 {
            let err = unsafe { GetLastError() };
            if err == 995 {
                // ERROR_OPERATION_ABORTED: nothing pending.
                return Ok(0);
            }
            return Err(io::Error::last_os_error());
        }
        Ok(read as usize)
    }

    pub fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut written = 0usize;
        while written < data.len() {
            let mut count = 0u32;
            let ok = unsafe {
                WriteFile(
                    self.handle,
                    data[written..].as_ptr(),
                    (data.len() - written).min(u32::MAX as usize) as u32,
                    &mut count,
                    std::ptr::null_mut(),
                )
            };
            if ok == 0 {
                return Err(io::Error::last_os_error());
            }
            written += count as usize;
            if count == 0 {
                if Instant::now() >= deadline {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "serial write timed out",
                    ));
                }
                std::thread::sleep(Duration::from_millis(2));
            }
        }
        Ok(())
    }
}

impl Drop for Port {
    fn drop(&mut self) {
        unsafe {
            CloseHandle(self.handle);
        }
    }
}

pub fn list_ports() -> Vec<PortInfo> {
    let mut out = Vec::new();
    let sub_key = b"HARDWARE\\DEVICEMAP\\SERIALCOMM\0";
    unsafe {
        let mut key: Hkey = std::ptr::null_mut();
        let rc = RegOpenKeyExA(
            hkey_local_machine(),
            sub_key.as_ptr() as *const c_char,
            0,
            KEY_READ,
            &mut key,
        );
        if rc != ERROR_SUCCESS {
            return out;
        }
        let mut index = 0u32;
        loop {
            let mut name = [0u8; 256];
            let mut name_len = name.len() as u32;
            let mut data = [0u8; 256];
            let mut data_len = data.len() as u32;
            let mut kind = 0u32;
            let rc = RegEnumValueA(
                key,
                index,
                name.as_mut_ptr() as *mut c_char,
                &mut name_len,
                std::ptr::null_mut(),
                &mut kind,
                data.as_mut_ptr(),
                &mut data_len,
            );
            if rc == ERROR_NO_MORE_ITEMS {
                break;
            }
            if rc != ERROR_SUCCESS {
                index += 1;
                continue;
            }
            let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
            let label = String::from_utf8_lossy(&data[..end]).trim().to_string();
            if label.to_ascii_uppercase().starts_with("COM")
                && !out.iter().any(|p: &PortInfo| p.path == label)
            {
                out.push(PortInfo {
                    path: label.clone(),
                    label,
                    kind: "usb",
                });
            }
            index += 1;
        }
        RegCloseKey(key);
    }
    out.sort_by_key(|p| {
        p.path
            .trim_start_matches("COM")
            .parse::<u32>()
            .unwrap_or(u32::MAX)
    });
    out
}

/// Readable serial port names for help messages.
pub fn default_port_hint() -> &'static str {
    "COMx (ST-Link Virtual COM Port)"
}
