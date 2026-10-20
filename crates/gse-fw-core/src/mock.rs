//! In-memory hardware mock used by unit tests and by `gse-sim`.
//!
//! Only compiled with the `mock` feature (or under `cargo test`); the flight
//! firmware never sees this module.

use std::collections::VecDeque;

use gse_proto::{Endpoint, Frame, FrameDecoder, Message};

use crate::hal::{GroundHal, Leds, VehicleHal};

/// Fake vehicle board: captures servo/igniter outputs and radio traffic.
pub struct MockVehicle {
    pub now_ms: u32,
    pub fill: bool,
    pub srv: bool,
    pub ign: bool,
    pub radio_in: VecDeque<u8>,
    pub radio_out: Vec<u8>,
    pub leds: Leds,
    pub watchdog_kicks: u32,
    pub device_id: u32,
    pub radio_ready: bool,
}

impl Default for MockVehicle {
    fn default() -> Self {
        MockVehicle {
            now_ms: 0,
            fill: false,
            srv: false,
            ign: false,
            radio_in: VecDeque::new(),
            radio_out: Vec::new(),
            leds: Leds::default(),
            watchdog_kicks: 0,
            device_id: 0x5645_4849, // "VEHI"
            radio_ready: true,
        }
    }
}

impl MockVehicle {
    pub fn advance(&mut self, ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(ms);
    }

    pub fn feed_bytes(&mut self, bytes: &[u8]) {
        self.radio_in.extend(bytes.iter().copied());
    }

    pub fn feed_frame(&mut self, message: &Message, src: Endpoint, dst: Endpoint, seq: u8) {
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let n = gse_proto::encode_message(message, src, dst, seq, &mut buf).expect("encode");
        self.radio_in.extend(buf[..n].iter().copied());
    }

    /// Parse every complete frame the board has transmitted and clear the buffer.
    pub fn drain_frames(&mut self) -> Vec<Frame> {
        let mut decoder = FrameDecoder::new();
        let mut frames = Vec::new();
        for &byte in &self.radio_out {
            if let Some(frame) = decoder.push(byte) {
                frames.push(frame);
            }
        }
        self.radio_out.clear();
        frames
    }
}

impl VehicleHal for MockVehicle {
    fn now_ms(&self) -> u32 {
        self.now_ms
    }

    fn set_fill(&mut self, on: bool) {
        self.fill = on;
    }

    fn set_srv(&mut self, on: bool) {
        self.srv = on;
    }

    fn set_ign(&mut self, on: bool) {
        self.ign = on;
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        let mut n = 0;
        while n < buf.len() {
            match self.radio_in.pop_front() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    fn write_radio(&mut self, data: &[u8]) {
        if self.radio_ready {
            self.radio_out.extend_from_slice(data);
        }
    }

    fn radio_ready(&mut self) -> bool {
        self.radio_ready
    }

    fn set_leds(&mut self, leds: Leds) {
        self.leds = leds;
    }

    fn kick_watchdog(&mut self) {
        self.watchdog_kicks = self.watchdog_kicks.wrapping_add(1);
    }

    fn device_id(&self) -> u32 {
        self.device_id
    }
}

/// Fake ground bridge board.
pub struct MockGround {
    pub now_ms: u32,
    pub usb_in: VecDeque<u8>,
    pub usb_out: Vec<u8>,
    pub radio_in: VecDeque<u8>,
    pub radio_out: Vec<u8>,
    pub leds: Leds,
    pub watchdog_kicks: u32,
    pub device_id: u32,
    pub usb_tx_pending: usize,
    pub radio_ready: bool,
}

impl Default for MockGround {
    fn default() -> Self {
        MockGround {
            now_ms: 0,
            usb_in: VecDeque::new(),
            usb_out: Vec::new(),
            radio_in: VecDeque::new(),
            radio_out: Vec::new(),
            leds: Leds::default(),
            watchdog_kicks: 0,
            device_id: 0x4752_4F55, // "GROU"
            usb_tx_pending: 0,
            radio_ready: true,
        }
    }
}

impl MockGround {
    pub fn advance(&mut self, ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(ms);
    }

    pub fn feed_usb_frame(&mut self, message: &Message, src: Endpoint, dst: Endpoint, seq: u8) {
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let n = gse_proto::encode_message(message, src, dst, seq, &mut buf).expect("encode");
        self.usb_in.extend(buf[..n].iter().copied());
    }

    pub fn feed_bytes_to_usb(&mut self, bytes: &[u8]) {
        self.usb_in.extend(bytes.iter().copied());
    }

    pub fn feed_radio_frame(&mut self, message: &Message, src: Endpoint, dst: Endpoint, seq: u8) {
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let n = gse_proto::encode_message(message, src, dst, seq, &mut buf).expect("encode");
        self.radio_in.extend(buf[..n].iter().copied());
    }

    pub fn drain_usb_frames(&mut self) -> Vec<Frame> {
        let mut decoder = FrameDecoder::new();
        let mut frames = Vec::new();
        for &byte in &self.usb_out {
            if let Some(frame) = decoder.push(byte) {
                frames.push(frame);
            }
        }
        self.usb_out.clear();
        frames
    }

    pub fn drain_radio_frames(&mut self) -> Vec<Frame> {
        let mut decoder = FrameDecoder::new();
        let mut frames = Vec::new();
        for &byte in &self.radio_out {
            if let Some(frame) = decoder.push(byte) {
                frames.push(frame);
            }
        }
        self.radio_out.clear();
        frames
    }
}

impl GroundHal for MockGround {
    fn now_ms(&self) -> u32 {
        self.now_ms
    }

    fn read_usb(&mut self, buf: &mut [u8]) -> usize {
        let mut n = 0;
        while n < buf.len() {
            match self.usb_in.pop_front() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    fn write_usb(&mut self, data: &[u8]) {
        self.usb_out.extend_from_slice(data);
    }

    fn usb_tx_pending(&self) -> usize {
        self.usb_tx_pending
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        let mut n = 0;
        while n < buf.len() {
            match self.radio_in.pop_front() {
                Some(b) => {
                    buf[n] = b;
                    n += 1;
                }
                None => break,
            }
        }
        n
    }

    fn write_radio(&mut self, data: &[u8]) {
        if self.radio_ready {
            self.radio_out.extend_from_slice(data);
        }
    }

    fn radio_ready(&mut self) -> bool {
        self.radio_ready
    }

    fn set_leds(&mut self, leds: Leds) {
        self.leds = leds;
    }

    fn kick_watchdog(&mut self) {
        self.watchdog_kicks = self.watchdog_kicks.wrapping_add(1);
    }

    fn device_id(&self) -> u32 {
        self.device_id
    }
}
