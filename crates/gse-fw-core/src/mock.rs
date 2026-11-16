//! In-memory hardware mock used by unit tests and by `gse-sim`.
//!
//! Only compiled with the `mock` feature (or under `cargo test`); the flight
//! firmware never sees this module.

use std::collections::VecDeque;

use gse_proto::{Endpoint, Frame, FrameDecoder, LinkConfig, Message, RadioModule, Reason};

use crate::hal::{GroundHal, Leds, LinkState, VehicleHal};
use crate::pad::{PadHal, PadServo};

/// An ESP-NOW board cannot switch to a LoRa module (or back) at run time.
fn probe_radio(active: RadioModule, wanted: RadioModule, e28_present: bool) -> bool {
    match wanted {
        RadioModule::DxLr42 => !active.is_esp_now(),
        RadioModule::E28_2G4M27S => e28_present && !active.is_esp_now(),
        RadioModule::EspNow => active.is_esp_now(),
    }
}

/// Fake vehicle board: captures servo/igniter outputs, radio traffic, and link settings.
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
    pub link: LinkState,
    pub e28_present: bool,
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
            link: LinkState::default(),
            e28_present: true,
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

    fn radio_tx_busy(&self) -> bool {
        !self.radio_out.is_empty()
    }

    fn link_state(&self) -> LinkState {
        self.link
    }

    fn probe_radio_module(&mut self, radio: RadioModule) -> bool {
        probe_radio(self.link.radio, radio, self.e28_present)
    }

    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        if cfg.radio == RadioModule::E28_2G4M27S && !self.e28_present {
            return Err(Reason::HardwareFault);
        }
        if cfg.radio.is_esp_now() != self.link.radio.is_esp_now() {
            return Err(Reason::NotPermitted);
        }
        self.link.uart_baud = cfg.uart_baud;
        self.link.radio = cfg.radio;
        if cfg.persist() {
            self.link.flash_saved = true;
        }
        Ok(cfg.persist())
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
    pub link: LinkState,
    pub e28_present: bool,
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
            link: LinkState::default(),
            e28_present: true,
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

    fn usb_tx_busy(&self) -> bool {
        self.usb_tx_pending > 0 || !self.usb_out.is_empty()
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

    fn radio_tx_busy(&self) -> bool {
        !self.radio_out.is_empty()
    }

    fn link_state(&self) -> LinkState {
        self.link
    }

    fn probe_radio_module(&mut self, radio: RadioModule) -> bool {
        probe_radio(self.link.radio, radio, self.e28_present)
    }

    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        if cfg.radio == RadioModule::E28_2G4M27S && !self.e28_present {
            return Err(Reason::HardwareFault);
        }
        if cfg.radio.is_esp_now() != self.link.radio.is_esp_now() {
            return Err(Reason::NotPermitted);
        }
        self.link.usb_baud = cfg.usb_baud;
        self.link.uart_baud = cfg.uart_baud;
        self.link.radio = cfg.radio;
        if cfg.persist() {
            self.link.flash_saved = true;
        }
        Ok(cfg.persist())
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

/// Fake ESP32 pad board (GSE-simple pad node): records servo pulses, the
/// igniter relay, the heartbeat pin and radio traffic.
pub struct MockPad {
    pub now_ms: u32,
    /// Last pulse width per [`PadServo`] (0 = never driven).
    pub servo_us: [u16; 4],
    pub ign: bool,
    pub heartbeat: bool,
    pub heartbeat_toggles: u32,
    pub radio_in: VecDeque<u8>,
    pub radio_out: Vec<u8>,
    pub leds: Leds,
    pub watchdog_kicks: u32,
    pub device_id: u32,
    pub radio_ready: bool,
    pub rssi_dbm: Option<i8>,
    /// Radio link; ESP-NOW by default, a LoRa module for the NUCLEO pad.
    pub link: LinkState,
    pub e28_present: bool,
}

impl Default for MockPad {
    fn default() -> Self {
        MockPad {
            now_ms: 0,
            servo_us: [0; 4],
            ign: false,
            heartbeat: false,
            heartbeat_toggles: 0,
            radio_in: VecDeque::new(),
            radio_out: Vec::new(),
            leds: Leds::default(),
            watchdog_kicks: 0,
            device_id: 0x5041_4431, // "PAD1"
            radio_ready: true,
            rssi_dbm: Some(-48),
            link: LinkState {
                usb_baud: gse_proto::UsbBaud::Baud115200,
                uart_baud: gse_proto::DxLr42Baud::Baud115200,
                radio: RadioModule::EspNow,
                flash_saved: false,
            },
            e28_present: true,
        }
    }
}

impl MockPad {
    /// A NUCLEO pad on a LoRa module (`DX-LR42` or `E28`).
    pub fn lora(radio: RadioModule, uart_baud: gse_proto::DxLr42Baud) -> Self {
        let mut pad = MockPad::default();
        pad.link.radio = radio;
        pad.link.uart_baud = uart_baud;
        pad.rssi_dbm = None;
        pad
    }
}

impl MockPad {
    pub fn advance(&mut self, ms: u32) {
        self.now_ms = self.now_ms.wrapping_add(ms);
    }

    pub fn servo(&self, servo: PadServo) -> u16 {
        self.servo_us[servo as usize]
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

impl PadHal for MockPad {
    fn now_ms(&self) -> u32 {
        self.now_ms
    }

    fn set_servo_us(&mut self, servo: PadServo, pulse_us: u16) {
        self.servo_us[servo as usize] = pulse_us;
    }

    fn set_ign(&mut self, on: bool) {
        self.ign = on;
    }

    fn set_heartbeat(&mut self, level: bool) {
        if level != self.heartbeat {
            self.heartbeat_toggles = self.heartbeat_toggles.wrapping_add(1);
        }
        self.heartbeat = level;
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

    fn rssi_dbm(&self) -> Option<i8> {
        self.rssi_dbm
    }

    fn link_state(&self) -> LinkState {
        self.link
    }

    fn radio_tx_busy(&self) -> bool {
        !self.radio_out.is_empty()
    }

    fn probe_radio_module(&mut self, radio: RadioModule) -> bool {
        probe_radio(self.link.radio, radio, self.e28_present)
    }

    fn apply_link_config(&mut self, cfg: LinkConfig) -> Result<bool, Reason> {
        if cfg.radio.is_esp_now() != self.link.radio.is_esp_now() {
            return Err(Reason::NotPermitted);
        }
        if cfg.radio == RadioModule::E28_2G4M27S && !self.e28_present {
            return Err(Reason::HardwareFault);
        }
        self.link.uart_baud = cfg.uart_baud;
        self.link.radio = cfg.radio;
        if cfg.persist() {
            self.link.flash_saved = true;
        }
        Ok(cfg.persist())
    }
}
