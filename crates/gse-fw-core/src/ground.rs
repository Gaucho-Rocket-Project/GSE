//! Ground (bridge) NUCLEO-H753ZI logic.
//!
//! The ground board is deliberately "dumb": it validates framing on both links
//! and forwards frames between USB and the SX1281 UART. It only answers frames
//! addressed to itself (used by the app's port auto-detection) and injects its
//! own HELLO / LINKSTAT telemetry. All safety logic lives on the vehicle and in
//! the ground app.
//!
//! Address filtering is what keeps the two links apart:
//!
//! * USB -> radio: everything except frames addressed to GROUND
//! * radio -> USB: frames addressed to HOST or BROADCAST, plus local ones

use gse_proto::{
    encode_message, Endpoint, Event, EventCode, Frame, FrameDecoder, Heartbeat, Hello, LinkStat,
    Message, Pong, StateSnapshot,
};

use crate::hal::{deadline_passed, elapsed_ms, GroundHal, Leds};

/// HELLO rate towards the PC.
pub const HELLO_PERIOD_MS: u32 = 1_000;
/// LINKSTAT rate towards the PC.
pub const LINKSTAT_PERIOD_MS: u32 = 1_000;
/// If more than this many bytes are queued for USB, drop radio frames instead
/// of blocking the radio receiver (a wedged PC must not stall the bridge).
pub const USB_BACKPRESSURE_BYTES: usize = 2_048;
/// Received-signal telemetry is reported as "age of the last radio frame".
pub const RADIO_AGE_CAP_MS: u32 = 60_000;
const RX_CHUNK: usize = 128;

pub struct Ground<H: GroundHal> {
    hal: H,
    usb: FrameDecoder,
    radio: FrameDecoder,
    tx_seq: u8,
    boot_ms: u32,
    booted: bool,
    saw_host: bool,
    last_usb_activity: u32,
    last_radio_activity: u32,
    have_radio: bool,
    last_hello_tx: u32,
    last_linkstat_tx: u32,
    frames_from_radio: u32,
    frames_to_radio: u32,
    crc_errors: u16,
    uart_errors: u16,
    drops: u16,
}

impl<H: GroundHal> Ground<H> {
    pub fn new(hal: H) -> Self {
        let now = hal.now_ms();
        Ground {
            hal,
            usb: FrameDecoder::new(),
            radio: FrameDecoder::new(),
            tx_seq: 0,
            boot_ms: now,
            booted: false,
            saw_host: false,
            last_usb_activity: now,
            last_radio_activity: now,
            have_radio: false,
            last_hello_tx: now,
            last_linkstat_tx: now,
            frames_from_radio: 0,
            frames_to_radio: 0,
            crc_errors: 0,
            uart_errors: 0,
            drops: 0,
        }
    }

    pub fn hal(&self) -> &H {
        &self.hal
    }

    pub fn hal_mut(&mut self) -> &mut H {
        &mut self.hal
    }

    pub fn decoder_stats(&self) -> (gse_proto::DecodeStats, gse_proto::DecodeStats) {
        (self.usb.stats(), self.radio.stats())
    }

    pub fn poll(&mut self) {
        let now = self.hal.now_ms();
        if !self.booted {
            self.booted = true;
            let state = StateSnapshot::all_safe();
            self.send_event(
                EventCode::BridgeReady,
                0,
                now,
                state,
            );
            self.send_hello(now);
        }

        let mut buf = [0u8; RX_CHUNK];

        // USB (PC) -> radio.
        for _ in 0..8 {
            let n = self.hal.read_usb(&mut buf);
            if n == 0 {
                break;
            }
            self.saw_host = true;
            self.last_usb_activity = now;
            for &byte in &buf[..n] {
                if let Some(frame) = self.usb.push(byte) {
                    self.handle_usb_frame(frame, now);
                }
            }
        }

        // Radio -> USB.
        for _ in 0..8 {
            let n = self.hal.read_radio(&mut buf);
            if n == 0 {
                break;
            }
            self.last_radio_activity = now;
            self.have_radio = true;
            for &byte in &buf[..n] {
                if let Some(frame) = self.radio.push(byte) {
                    self.handle_radio_frame(frame, now);
                }
            }
        }

        if elapsed_ms(now, self.last_hello_tx) >= HELLO_PERIOD_MS {
            self.send_hello(now);
        }
        if self.saw_host && elapsed_ms(now, self.last_linkstat_tx) >= LINKSTAT_PERIOD_MS {
            self.send_linkstat(now);
        }

        let usb_stats = self.usb.stats();
        let radio_stats = self.radio.stats();
        self.crc_errors = (usb_stats.crc_errors + radio_stats.crc_errors).min(u16::MAX as u32) as u16;
        self.uart_errors =
            (usb_stats.malformed + radio_stats.malformed).min(u16::MAX as u32) as u16;

        let host_up = self.saw_host && elapsed_ms(now, self.last_usb_activity) < 2_000;
        let radio_active = self.have_radio && elapsed_ms(now, self.last_radio_activity) < 2_000;
        self.hal.set_leds(Leds {
            link: host_up,
            activity: elapsed_ms(now, self.last_usb_activity) < 250
                || elapsed_ms(now, self.last_radio_activity) < 250,
            fault: host_up && !radio_active,
        });
        self.hal.kick_watchdog();
    }

    fn handle_usb_frame(&mut self, frame: Frame, now: u32) {
        if frame.dst == Endpoint::Ground {
            self.handle_local(frame, now, true);
            return;
        }
        if frame.dst == Endpoint::Host {
            return;
        }
        if self.hal.radio_ready() {
            let mut buf = [0u8; Frame::MAX_ENCODED];
            if let Some(n) = frame.encode(&mut buf) {
                self.hal.write_radio(&buf[..n]);
                self.frames_to_radio = self.frames_to_radio.wrapping_add(1);
            }
        } else {
            self.drops = self.drops.wrapping_add(1);
        }
    }

    fn handle_radio_frame(&mut self, frame: Frame, now: u32) {
        self.frames_from_radio = self.frames_from_radio.wrapping_add(1);
        if frame.dst == Endpoint::Ground {
            self.handle_local(frame, now, false);
            return;
        }
        if frame.dst != Endpoint::Host && frame.dst != Endpoint::Broadcast {
            return;
        }
        if self.hal.usb_tx_pending() > USB_BACKPRESSURE_BYTES {
            self.drops = self.drops.wrapping_add(1);
            return;
        }
        let mut buf = [0u8; Frame::MAX_ENCODED];
        if let Some(n) = frame.encode(&mut buf) {
            self.hal.write_usb(&buf[..n]);
        }
    }

    fn handle_local(&mut self, frame: Frame, now: u32, from_usb: bool) {
        match frame.message() {
            Some(Message::Ping(ping)) => {
                let uptime = elapsed_ms(now, self.boot_ms);
                let pong = Message::Pong(Pong {
                    nonce: ping.nonce,
                    uptime_ms: uptime,
                    state: StateSnapshot::all_safe(),
                });
                if from_usb {
                    self.send_usb(&pong);
                } else {
                    self.send_radio(&pong, Endpoint::Host);
                }
            }
            Some(Message::Heartbeat(Heartbeat { .. })) => {}
            _ => {}
        }
    }

    fn send_hello(&mut self, now: u32) {
        let hello = Hello {
            protocol: gse_proto::PROTOCOL_VERSION,
            fw: {
                let (a, b, c) = gse_proto::FIRMWARE_VERSION;
                [a, b, c]
            },
            role: Endpoint::Ground,
            uptime_ms: elapsed_ms(now, self.boot_ms),
            device_id: self.hal.device_id(),
            capabilities: gse_proto::capabilities::RADIO
                | gse_proto::capabilities::USB_BRIDGE
                | gse_proto::capabilities::WATCHDOG,
        };
        if self.send_usb_msg(Message::Hello(hello)) {
            self.last_hello_tx = now;
        } else {
            self.last_hello_tx = now.wrapping_sub(HELLO_PERIOD_MS - 20);
        }
    }

    fn send_linkstat(&mut self, now: u32) {
        let age = if self.have_radio {
            elapsed_ms(now, self.last_radio_activity).min(RADIO_AGE_CAP_MS)
        } else {
            RADIO_AGE_CAP_MS
        };
        let stat = LinkStat {
            frames_from_radio: self.frames_from_radio,
            frames_to_radio: self.frames_to_radio,
            crc_errors: self.crc_errors,
            uart_errors: self.uart_errors,
            bridge_uptime_ms: elapsed_ms(now, self.boot_ms),
            last_radio_age_ms: age.min(u16::MAX as u32) as u16,
            drops: self.drops,
        };
        if self.send_usb_msg(Message::LinkStat(stat)) {
            self.last_linkstat_tx = now;
        }
    }

    fn send_event(&mut self, code: EventCode, detail: u8, now: u32, state: StateSnapshot) {
        let event = Event {
            code,
            detail,
            t_ms: elapsed_ms(now, self.boot_ms),
            state,
        };
        self.send_usb_msg(Message::Event(event));
    }

    fn send_usb_msg(&mut self, message: Message) -> bool {
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let seq = self.tx_seq;
        match encode_message(&message, Endpoint::Ground, Endpoint::Host, seq, &mut buf) {
            Some(n) => {
                self.hal.write_usb(&buf[..n]);
                self.tx_seq = seq.wrapping_add(1);
                true
            }
            None => false,
        }
    }

    fn send_usb(&mut self, message: &Message) {
        self.send_usb_msg(*message);
    }

    fn send_radio(&mut self, message: &Message, dst: Endpoint) -> bool {
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let seq = self.tx_seq;
        match encode_message(message, Endpoint::Ground, dst, seq, &mut buf) {
            Some(n) => {
                if !self.hal.radio_ready() {
                    return false;
                }
                self.hal.write_radio(&buf[..n]);
                self.tx_seq = seq.wrapping_add(1);
                true
            }
            None => false,
        }
    }
}

/// True when the age counter has wrapped past `age`.
pub fn radio_age_exceeded(now: u32, last: u32, age: u32) -> bool {
    deadline_passed(now, last.wrapping_add(age))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockGround;
    use gse_proto::{Ack, AckResult, Command, Control, Ping, Reason};

    fn new_ground() -> Ground<MockGround> {
        Ground::new(MockGround::default())
    }

    #[test]
    fn boots_and_announces_hello() {
        let mut ground = new_ground();
        ground.poll();
        let frames = ground.hal_mut().drain_usb_frames();
        assert!(frames.iter().any(|f| matches!(f.message(), Some(Message::Hello(h)) if h.role == Endpoint::Ground)));
        assert!(frames.iter().any(|f| matches!(f.message(), Some(Message::Event(e)) if e.code == EventCode::BridgeReady)));
        assert!(ground.hal().watchdog_kicks > 0);
    }

    #[test]
    fn forwards_host_command_to_radio() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().drain_radio_frames();
        let cmd = Command {
            cmd_id: 1,
            control: Control::Fill,
            action: 1,
            arg_s: 60,
            flags: 0,
        };
        ground
            .hal_mut()
            .feed_usb_frame(&Message::Command(cmd), Endpoint::Host, Endpoint::Vehicle, 1);
        ground.poll();
        let frames = ground.hal_mut().drain_radio_frames();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].message(), Some(Message::Command(cmd)));
        assert_eq!(frames[0].src, Endpoint::Host);
        assert_eq!(frames[0].dst, Endpoint::Vehicle);
    }

    #[test]
    fn forwards_vehicle_ack_to_usb() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().drain_usb_frames();
        let ack = Ack {
            cmd_id: 5,
            control: Control::Srv,
            action: 1,
            result: AckResult::Ok,
            reason: Reason::None,
            exec_ms: 310,
            state: StateSnapshot::all_safe(),
        };
        ground
            .hal_mut()
            .feed_radio_frame(&Message::Ack(ack), Endpoint::Vehicle, Endpoint::Host, 2);
        ground.poll();
        let frames = ground.hal_mut().drain_usb_frames();
        assert!(frames.iter().any(|f| f.message() == Some(Message::Ack(ack))));
    }

    #[test]
    fn answers_ping_addressed_to_ground() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().drain_usb_frames();
        ground.hal_mut().feed_usb_frame(
            &Message::Ping(Ping {
                nonce: 0xDEAD,
                host_ms: 1,
            }),
            Endpoint::Host,
            Endpoint::Ground,
            1,
        );
        ground.poll();
        let frames = ground.hal_mut().drain_usb_frames();
        let pong = frames.iter().find_map(|f| match f.message() {
            Some(Message::Pong(p)) => Some(p),
            _ => None,
        });
        assert_eq!(pong.expect("pong").nonce, 0xDEAD);
        assert!(ground.hal().radio_out.is_empty(), "ping must not be forwarded");
    }

    #[test]
    fn ignores_frames_not_for_host() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().drain_usb_frames();
        ground.hal_mut().feed_radio_frame(
            &Message::State(StateSnapshot::all_safe()),
            Endpoint::Vehicle,
            Endpoint::Vehicle,
            1,
        );
        ground.poll();
        assert!(ground.hal_mut().drain_usb_frames().is_empty());
    }

    #[test]
    fn sends_linkstat_after_host_activity() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().drain_usb_frames();
        ground.hal_mut().feed_usb_frame(
            &Message::Heartbeat(Heartbeat {
                uptime_ms: 1,
                mode: 0,
            }),
            Endpoint::Host,
            Endpoint::Vehicle,
            1,
        );
        ground.poll();
        ground.hal_mut().advance(LINKSTAT_PERIOD_MS + 10);
        ground.poll();
        let frames = ground.hal_mut().drain_usb_frames();
        let stat = frames.iter().find_map(|f| match f.message() {
            Some(Message::LinkStat(s)) => Some(s),
            _ => None,
        });
        assert!(stat.is_some());
        assert_eq!(stat.unwrap().frames_to_radio, 1);
    }

    #[test]
    fn counts_corruption_without_forwarding() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().drain_radio_frames();
        ground.hal_mut().drain_usb_frames();
        ground.hal_mut().feed_bytes_to_usb(&[0x00, 0x01, 0x02, 0x00]);
        ground.poll();
        assert!(ground.hal_mut().drain_radio_frames().is_empty());
        assert!(ground.hal_mut().drain_usb_frames().is_empty());
        assert!(ground.usb.stats().malformed > 0);
    }

    #[test]
    fn no_forwarding_when_radio_not_ready() {
        let mut ground = new_ground();
        ground.poll();
        ground.hal_mut().radio_ready = false;
        let cmd = Command {
            cmd_id: 1,
            control: Control::Fill,
            action: 1,
            arg_s: 60,
            flags: 0,
        };
        ground
            .hal_mut()
            .feed_usb_frame(&Message::Command(cmd), Endpoint::Host, Endpoint::Vehicle, 1);
        ground.poll();
        assert!(ground.hal_mut().drain_radio_frames().is_empty());
        assert_eq!(ground.drops, 1);
    }
}
