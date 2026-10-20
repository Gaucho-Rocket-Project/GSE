//! Minimal Ctrl+C handling so the app can shut the engine down cleanly.
//!
//! `std` has no signal API and the project deliberately has no dependencies,
//! so this uses the platform primitives directly.

use std::sync::atomic::{AtomicBool, Ordering};

static REQUESTED: AtomicBool = AtomicBool::new(false);

pub fn requested() -> bool {
    REQUESTED.load(Ordering::SeqCst)
}

pub fn request() {
    REQUESTED.store(true, Ordering::SeqCst);
}

#[cfg(unix)]
mod imp {
    use super::*;

    pub const SIGINT: i32 = 2;
    pub const SIGTERM: i32 = 15;

    extern "C" {
        fn signal(sig: i32, handler: usize) -> usize;
    }

    extern "C" fn on_signal(_sig: i32) {
        REQUESTED.store(true, Ordering::SeqCst);
    }

    pub fn install() {
        let handler = on_signal as extern "C" fn(i32) as usize;
        unsafe {
            signal(SIGINT, handler);
            signal(SIGTERM, handler);
        }
    }
}

#[cfg(windows)]
mod imp {
    use super::*;

    #[link(name = "kernel32")]
    extern "system" {
        fn SetConsoleCtrlHandler(
            handler: Option<extern "system" fn(u32) -> i32>,
            add: i32,
        ) -> i32;
    }

    extern "system" fn on_ctrl(_ctrl_type: u32) -> i32 {
        REQUESTED.store(true, Ordering::SeqCst);
        1
    }

    pub fn install() {
        unsafe {
            SetConsoleCtrlHandler(Some(on_ctrl), 1);
        }
    }
}

#[cfg(not(any(unix, windows)))]
mod imp {
    pub fn install() {}
}

pub fn install() {
    imp::install();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn installing_handlers_is_safe() {
        // Must never panic, even when no console is attached.
        install();
        let before = requested();
        assert_eq!(before, requested());
    }
}
