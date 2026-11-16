//! ESP-NOW adapters for the shared GSE ground and vehicle state machines.
//!
//! ESP-NOW delivers complete datagrams, while `gse-fw-core` consumes a byte
//! stream. The adapters therefore queue complete packets and expose them to the
//! core one at a time. Each packet is still a normal `gse-proto` COBS frame;
//! ESP-NOW is only the transport.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use gse_fw_core::hal::{GroundHal, Leds, LinkState, VehicleHal};
use gse_proto::{LinkConfig, Reason};

pub const ESPNOW_MTU: usize = 250;
pub const RX_QUEUE_LEN: usize = 8;

/// Small allocation-free packet queue for an ESP-NOW receive callback.
///
/// ESP-NOW callbacks must not run the protocol state machine directly. A
/// platform callback should call `push`, then the main loop should provide the
/// queue to an `EspNowIo` implementation and let the shared firmware poll it.
#[derive(Clone, Copy)]
pub struct PacketQueue {
    packets: [[u8; ESPNOW_MTU]; RX_QUEUE_LEN],
    lengths: [u16; RX_QUEUE_LEN],
    read: usize,
    write: usize,
    count: usize,
    dropped: u32,
}

impl PacketQueue {
    pub const fn new() -> Self {
        Self {
            packets: [[0; ESPNOW_MTU]; RX_QUEUE_LEN],
            lengths: [0; RX_QUEUE_LEN],
            read: 0,
            write: 0,
            count: 0,
            dropped: 0,
        }
    }

    pub fn push(&mut self, packet: &[u8]) -> bool {
        if packet.len() > ESPNOW_MTU || self.count == RX_QUEUE_LEN {
            self.dropped = self.dropped.saturating_add(1);
            return false;
        }
        self.packets[self.write][..packet.len()].copy_from_slice(packet);
        self.lengths[self.write] = packet.len() as u16;
        self.write = (self.write + 1) % RX_QUEUE_LEN;
        self.count += 1;
        true
    }

    pub fn pop(&mut self, out: &mut [u8]) -> usize {
        if self.count == 0 {
            return 0;
        }
        let len = self.lengths[self.read] as usize;
        if out.len() < len {
            return 0;
        }
        out[..len].copy_from_slice(&self.packets[self.read][..len]);
        self.read = (self.read + 1) % RX_QUEUE_LEN;
        self.count -= 1;
        len
    }

    pub const fn pending(&self) -> usize {
        self.count
    }

    pub const fn dropped(&self) -> u32 {
        self.dropped
    }
}

impl Default for PacketQueue {
    fn default() -> Self {
        Self::new()
    }
}

/// Platform operations required from an ESP32 implementation.
pub trait EspNowIo {
    fn now_ms(&self) -> u32;
    fn receive(&mut self, out: &mut [u8]) -> usize;
    fn send(&mut self, data: &[u8]) -> bool;
    fn tx_busy(&self) -> bool {
        false
    }
    fn set_leds(&mut self, leds: Leds);
    fn kick_watchdog(&mut self);
    fn device_id(&self) -> u32;
}

/// Vehicle-specific GPIO/PWM operations.
pub trait VehicleOutputs {
    fn set_fill(&mut self, on: bool);
    fn set_srv(&mut self, on: bool);
    fn set_ign(&mut self, on: bool);
}

/// A vehicle HAL using ESP-NOW datagrams for the RF link.
pub struct EspNowVehicleHal<I, O> {
    pub io: I,
    pub outputs: O,
}

impl<I, O> EspNowVehicleHal<I, O> {
    pub const fn new(io: I, outputs: O) -> Self {
        Self { io, outputs }
    }
}

impl<I: EspNowIo, O: VehicleOutputs> VehicleHal for EspNowVehicleHal<I, O> {
    fn now_ms(&self) -> u32 {
        self.io.now_ms()
    }

    fn set_fill(&mut self, on: bool) {
        self.outputs.set_fill(on);
    }

    fn set_srv(&mut self, on: bool) {
        self.outputs.set_srv(on);
    }

    fn set_ign(&mut self, on: bool) {
        self.outputs.set_ign(on);
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        self.io.receive(buf)
    }

    fn write_radio(&mut self, data: &[u8]) {
        let _ = self.io.send(data);
    }

    fn radio_ready(&mut self) -> bool {
        !self.io.tx_busy()
    }

    fn radio_tx_busy(&self) -> bool {
        self.io.tx_busy()
    }

    fn set_leds(&mut self, leds: Leds) {
        self.io.set_leds(leds);
    }

    fn kick_watchdog(&mut self) {
        self.io.kick_watchdog();
    }

    fn device_id(&self) -> u32 {
        self.io.device_id()
    }

    fn link_state(&self) -> LinkState {
        LinkState::default()
    }

    fn probe_radio_module(&mut self, _radio: gse_proto::RadioModule) -> bool {
        true
    }

    fn apply_link_config(&mut self, _cfg: LinkConfig) -> Result<bool, Reason> {
        Err(Reason::NotPermitted)
    }
}

/// A ground HAL using USB serial for the host side and ESP-NOW for RF.
pub trait UsbIo {
    fn read(&mut self, out: &mut [u8]) -> usize;
    fn write(&mut self, data: &[u8]);
    fn pending(&self) -> usize {
        0
    }
    fn busy(&self) -> bool {
        self.pending() != 0
    }
}

pub struct EspNowGroundHal<I, U> {
    pub io: I,
    pub usb: U,
}

impl<I, U> EspNowGroundHal<I, U> {
    pub const fn new(io: I, usb: U) -> Self {
        Self { io, usb }
    }
}

impl<I: EspNowIo, U: UsbIo> GroundHal for EspNowGroundHal<I, U> {
    fn now_ms(&self) -> u32 {
        self.io.now_ms()
    }

    fn read_usb(&mut self, buf: &mut [u8]) -> usize {
        self.usb.read(buf)
    }

    fn write_usb(&mut self, data: &[u8]) {
        self.usb.write(data);
    }

    fn usb_tx_pending(&self) -> usize {
        self.usb.pending()
    }

    fn usb_tx_busy(&self) -> bool {
        self.usb.busy()
    }

    fn read_radio(&mut self, buf: &mut [u8]) -> usize {
        self.io.receive(buf)
    }

    fn write_radio(&mut self, data: &[u8]) {
        let _ = self.io.send(data);
    }

    fn radio_ready(&mut self) -> bool {
        !self.io.tx_busy()
    }

    fn radio_tx_busy(&self) -> bool {
        self.io.tx_busy()
    }

    fn set_leds(&mut self, leds: Leds) {
        self.io.set_leds(leds);
    }

    fn kick_watchdog(&mut self) {
        self.io.kick_watchdog();
    }

    fn device_id(&self) -> u32 {
        self.io.device_id()
    }

    fn link_state(&self) -> LinkState {
        LinkState::default()
    }

    fn probe_radio_module(&mut self, _radio: gse_proto::RadioModule) -> bool {
        true
    }

    fn apply_link_config(&mut self, _cfg: LinkConfig) -> Result<bool, Reason> {
        Err(Reason::NotPermitted)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gse_fw_core::hal::{GroundHal, VehicleHal};

    struct Io {
        rx: [u8; ESPNOW_MTU],
        rx_len: usize,
        sent: usize,
    }

    impl EspNowIo for Io {
        fn now_ms(&self) -> u32 {
            0
        }
        fn receive(&mut self, out: &mut [u8]) -> usize {
            out[..self.rx_len].copy_from_slice(&self.rx[..self.rx_len]);
            let n = self.rx_len;
            self.rx_len = 0;
            n
        }
        fn send(&mut self, data: &[u8]) -> bool {
            self.sent = data.len();
            true
        }
        fn set_leds(&mut self, _: Leds) {}
        fn kick_watchdog(&mut self) {}
        fn device_id(&self) -> u32 {
            1
        }
    }

    struct Outputs;
    impl VehicleOutputs for Outputs {
        fn set_fill(&mut self, _: bool) {}
        fn set_srv(&mut self, _: bool) {}
        fn set_ign(&mut self, _: bool) {}
    }

    struct Usb;
    impl UsbIo for Usb {
        fn read(&mut self, _: &mut [u8]) -> usize {
            0
        }
        fn write(&mut self, _: &[u8]) {}
    }

    #[test]
    fn vehicle_maps_one_datagram_to_one_radio_read() {
        let mut io = Io {
            rx: [0; ESPNOW_MTU],
            rx_len: 3,
            sent: 0,
        };
        io.rx[..3].copy_from_slice(b"abc");
        let mut hal = EspNowVehicleHal::new(io, Outputs);
        let mut out = [0; 8];
        assert_eq!(hal.read_radio(&mut out), 3);
        assert_eq!(&out[..3], b"abc");
    }

    #[test]
    fn ground_writes_radio_as_one_datagram() {
        let io = Io {
            rx: [0; ESPNOW_MTU],
            rx_len: 0,
            sent: 0,
        };
        let mut hal = EspNowGroundHal::new(io, Usb);
        hal.write_radio(b"abc");
        assert_eq!(hal.io.sent, 3);
    }

    #[test]
    fn queue_preserves_datagram_boundaries_and_drops_when_full() {
        let mut queue = PacketQueue::new();
        for _ in 0..RX_QUEUE_LEN {
            assert!(queue.push(b"packet"));
        }
        assert!(!queue.push(b"overflow"));
        assert_eq!(queue.dropped(), 1);
        let mut out = [0; ESPNOW_MTU];
        for _ in 0..RX_QUEUE_LEN {
            assert_eq!(queue.pop(&mut out), 6);
            assert_eq!(&out[..6], b"packet");
        }
        assert_eq!(queue.pending(), 0);
    }
}
