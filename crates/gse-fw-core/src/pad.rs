//! GSE-simple pad node for the ESP32 + ESP-NOW vehicle board.
//!
//! A port of `GSE-simple/firmware/src/pad/pad_node.cpp` onto the GSE frame
//! layer. The control algorithm is unchanged:
//!
//! * every 20 ms the outputs are recomputed from the last valid
//!   [`PadCommand`] and its age by the pure [`gse_proto::pad::failsafe`],
//! * no valid command for the watchdog (1.0 s default) -> FILL closed,
//!   PURGE closed, VEHICLE open, IGN off,
//! * the igniter needs the arm code in every command and fires once per press
//!   for at most `ign_max_ds`; it must be released before it can fire again,
//! * PURGE is forced closed unless the host enabled it in [`PadConfig`],
//! * the outputs actually driven are reported in [`PadState`] at 10 Hz.
//!
//! The two vehicle-valve servos (fuel and oxidizer) both follow the `veh`
//! command; each has its own pin, direction and trim ([`PadBoard`]).

use gse_proto::pad::{
    self, clamp_ign_max_ds, clamp_travel, clamp_wdg_ds, failsafe, servo_pulse_us, state_flags,
    PadCommand, PadConfig, PadEvent, PadEventCode, PadOutputs, PadRunState, PadState,
};
use gse_proto::{
    capabilities, encode_message, flags, Endpoint, Frame, FrameDecoder, Hello, Message, Pong,
    StateSnapshot, FIRMWARE_VERSION, PROTOCOL_VERSION,
};

use crate::hal::{elapsed_ms, Leds};

const RX_CHUNK: usize = 128;
/// LED "activity" window after the last valid command.
const ACTIVITY_MS: u32 = 250;

/// The four servo outputs on the pad board.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PadServo {
    Fill = 0,
    /// First vehicle valve (fuel).
    Veh = 1,
    /// Second vehicle valve (oxidizer); follows the same `veh` command.
    Veh2 = 2,
    Purge = 3,
}

impl PadServo {
    pub const ALL: [PadServo; 4] = [
        PadServo::Fill,
        PadServo::Veh,
        PadServo::Veh2,
        PadServo::Purge,
    ];

    pub const fn label(self) -> &'static str {
        match self {
            PadServo::Fill => "FILL",
            PadServo::Veh => "VEH",
            PadServo::Veh2 => "VEH2",
            PadServo::Purge => "PURGE",
        }
    }
}

/// Mechanical setup of one servo output (GSE-simple `GSE_SERVO_*_REV/_TRIM_US`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServoSetup {
    /// `false` when no servo is wired to this output (it is never driven).
    pub fitted: bool,
    /// Mirror the open/closed direction.
    pub reversed: bool,
    /// Neutral offset in microseconds.
    pub trim_us: i16,
}

impl ServoSetup {
    pub const FITTED: ServoSetup = ServoSetup {
        fitted: true,
        reversed: false,
        trim_us: 0,
    };
}

/// Board-specific servo setup, indexed by [`PadServo`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PadBoard {
    pub servos: [ServoSetup; 4],
}

impl Default for PadBoard {
    fn default() -> Self {
        PadBoard {
            servos: [ServoSetup::FITTED; 4],
        }
    }
}

/// Hardware the pad core needs (implemented by the ESP32 firmware and mocks).
pub trait PadHal {
    /// Free-running millisecond counter. Wrapping is expected and handled.
    fn now_ms(&self) -> u32;
    /// Drive one servo with a 50 Hz pulse of `pulse_us` microseconds.
    fn set_servo_us(&mut self, servo: PadServo, pulse_us: u16);
    /// Igniter relay (true = energised).
    fn set_ign(&mut self, on: bool);
    /// Heartbeat pin feeding the hardware keep-alive circuit (toggled at 25 Hz).
    fn set_heartbeat(&mut self, _level: bool) {}
    /// Non-blocking read of received radio bytes.
    fn read_radio(&mut self, buf: &mut [u8]) -> usize;
    /// Send one frame over the radio (one ESP-NOW datagram).
    fn write_radio(&mut self, data: &[u8]);
    fn radio_ready(&mut self) -> bool {
        true
    }
    fn set_leds(&mut self, leds: Leds);
    fn kick_watchdog(&mut self);
    /// Unique-ish 32-bit board id reported in HELLO frames.
    fn device_id(&self) -> u32;
    /// RSSI of the last received datagram, if the radio reports it.
    fn rssi_dbm(&self) -> Option<i8> {
        None
    }
}

/// The pad node state machine.
pub struct Pad<H: PadHal> {
    hal: H,
    board: PadBoard,
    decoder: FrameDecoder,
    last_cmd: Option<PadCommand>,
    last_cmd_ms: u32,
    rx_count: u16,
    bad_cmds: u16,
    run_state: PadRunState,
    wdg_ds: u8,
    ign_max_ds: u8,
    travel: [u8; 3],
    allow_purge: bool,
    ign_on: bool,
    ign_since: u32,
    ign_spent: bool,
    outputs: PadOutputs,
    boot_ms: u32,
    booted: bool,
    last_tick_ms: u32,
    last_state_ms: u32,
    last_hello_ms: u32,
    heartbeat: bool,
    prev_comms_loss: bool,
    prev_armed: bool,
    tx_seq: u8,
}

impl<H: PadHal> Pad<H> {
    pub fn new(hal: H) -> Self {
        Self::with_board(hal, PadBoard::default())
    }

    pub fn with_board(hal: H, board: PadBoard) -> Self {
        let now = hal.now_ms();
        Pad {
            hal,
            board,
            decoder: FrameDecoder::new(),
            last_cmd: None,
            last_cmd_ms: now,
            rx_count: 0,
            bad_cmds: 0,
            run_state: PadRunState::WaitHello,
            wdg_ds: pad::DEFAULT_WDG_DS,
            ign_max_ds: pad::DEFAULT_IGN_MAX_DS,
            travel: [pad::DEFAULT_TRAVEL_DEG; 3],
            // ECR-01 hazard H8: purge stays forced closed until the operator
            // enables it for the session.
            allow_purge: false,
            ign_on: false,
            ign_since: now,
            ign_spent: false,
            outputs: PadOutputs::SAFE,
            boot_ms: now,
            booted: false,
            last_tick_ms: now,
            last_state_ms: now,
            last_hello_ms: now,
            heartbeat: false,
            prev_comms_loss: true,
            prev_armed: false,
            tx_seq: 0,
        }
    }

    pub fn hal(&self) -> &H {
        &self.hal
    }

    pub fn hal_mut(&mut self) -> &mut H {
        &mut self.hal
    }

    /// Outputs computed on the last 20 ms tick.
    pub fn outputs(&self) -> PadOutputs {
        self.outputs
    }

    pub fn ign_on(&self) -> bool {
        self.ign_on
    }

    pub fn ign_spent(&self) -> bool {
        self.ign_spent
    }

    pub fn allow_purge(&self) -> bool {
        self.allow_purge
    }

    pub fn poll(&mut self) {
        let now = self.hal.now_ms();
        if !self.booted {
            self.booted = true;
            // Safe outputs before anything else (GSE-simple setup()).
            self.outputs = PadOutputs::SAFE;
            self.drive_outputs();
            self.hal.set_ign(false);
            self.last_tick_ms = now;
            self.send_hello(now);
            self.send_event(PadEventCode::Boot, 0);
            // First STATE goes out on this pass.
            self.last_state_ms = now.wrapping_sub(pad::STATE_PERIOD_MS);
        }

        let mut buf = [0u8; RX_CHUNK];
        for _ in 0..8 {
            let n = self.hal.read_radio(&mut buf);
            if n == 0 {
                break;
            }
            for &byte in &buf[..n] {
                if let Some(frame) = self.decoder.push(byte) {
                    self.handle_frame(frame, now);
                }
            }
        }

        if elapsed_ms(now, self.last_tick_ms) >= pad::OUTPUT_TICK_MS {
            self.last_tick_ms = now;
            self.tick(now);
        }
        if elapsed_ms(now, self.last_state_ms) >= pad::STATE_PERIOD_MS {
            self.last_state_ms = now;
            self.send_state(now);
        }
        if elapsed_ms(now, self.last_hello_ms) >= pad::HELLO_PERIOD_MS {
            self.send_hello(now);
        }

        let active = self.last_cmd.is_some() && elapsed_ms(now, self.last_cmd_ms) < ACTIVITY_MS;
        self.hal.set_leds(Leds {
            link: !self.outputs.comms_loss,
            activity: active,
            fault: self.ign_on || self.last_cmd.is_none(),
        });
        self.hal.kick_watchdog();
    }

    fn handle_frame(&mut self, frame: Frame, now: u32) {
        if frame.dst != Endpoint::Vehicle && frame.dst != Endpoint::Broadcast {
            return;
        }
        if frame.src != Endpoint::Host && frame.src != Endpoint::Ground {
            return;
        }
        let message = match frame.message() {
            Some(m) => m,
            None => {
                self.bad_cmds = self.bad_cmds.wrapping_add(1);
                self.send_event(PadEventCode::BadCmd, frame.msg_type.as_u8());
                return;
            }
        };
        match message {
            Message::PadCmd(cmd) => {
                self.last_cmd = Some(cmd);
                self.last_cmd_ms = now;
                self.rx_count = self.rx_count.wrapping_add(1);
                if self.run_state == PadRunState::WaitHello {
                    self.run_state = PadRunState::Run;
                    self.send_event(PadEventCode::HelloOk, 0);
                }
                if cmd.wdg_ds != 0 {
                    self.wdg_ds = clamp_wdg_ds(cmd.wdg_ds);
                }
                if cmd.ign_max_ds != 0 {
                    self.ign_max_ds = clamp_ign_max_ds(cmd.ign_max_ds);
                }
            }
            Message::PadConfig(cfg) => {
                self.travel = [
                    clamp_travel(cfg.travel_fill),
                    clamp_travel(cfg.travel_veh),
                    clamp_travel(cfg.travel_prg),
                ];
                self.wdg_ds = clamp_wdg_ds(if cfg.wdg_ds != 0 {
                    cfg.wdg_ds
                } else {
                    pad::DEFAULT_WDG_DS
                });
                self.ign_max_ds = clamp_ign_max_ds(if cfg.ign_max_ds != 0 {
                    cfg.ign_max_ds
                } else {
                    pad::DEFAULT_IGN_MAX_DS
                });
                self.allow_purge = cfg.allow_purge;
                self.send_event(PadEventCode::CfgChanged, cfg.cfg_seq);
            }
            Message::Ping(ping) if frame.src == Endpoint::Host => {
                let pong = Message::Pong(Pong {
                    nonce: ping.nonce,
                    uptime_ms: elapsed_ms(now, self.boot_ms),
                    state: self.snapshot(now),
                });
                self.send(&pong);
            }
            _ => {}
        }
    }

    /// GSE-simple 20 ms output pass.
    fn tick(&mut self, now: u32) {
        let age = self
            .last_cmd
            .as_ref()
            .map(|_| elapsed_ms(now, self.last_cmd_ms));
        let outputs = failsafe(self.last_cmd.as_ref(), age, self.wdg_ds);
        if outputs.comms_loss && !self.prev_comms_loss && self.last_cmd.is_some() {
            self.send_event(PadEventCode::CommsLoss, 0);
        }
        if outputs.armed != self.prev_armed {
            self.send_event(
                if outputs.armed {
                    PadEventCode::Arm
                } else {
                    PadEventCode::Disarm
                },
                0,
            );
        }
        self.prev_comms_loss = outputs.comms_loss;
        self.prev_armed = outputs.armed;
        self.outputs = outputs;
        self.drive_outputs();
        self.service_igniter(outputs.ign, now);
        self.heartbeat = !self.heartbeat;
        self.hal.set_heartbeat(self.heartbeat);
    }

    fn drive_outputs(&mut self) {
        let o = self.outputs;
        let prg = self.allow_purge && o.prg;
        for servo in PadServo::ALL {
            let (open, travel) = match servo {
                PadServo::Fill => (o.fill, self.travel[0]),
                PadServo::Veh | PadServo::Veh2 => (o.veh, self.travel[1]),
                PadServo::Purge => (prg, self.travel[2]),
            };
            let setup = self.board.servos[servo as usize];
            if !setup.fitted {
                continue;
            }
            let us = servo_pulse_us(open, travel, setup.reversed, setup.trim_us);
            self.hal.set_servo_us(servo, us);
        }
    }

    /// One shot per press: request 0 -> off and re-armed; request 1 -> on until
    /// the on-time limit, then off until the request is released.
    fn service_igniter(&mut self, request: bool, now: u32) {
        if !request {
            self.ign_spent = false;
            if self.ign_on {
                self.ign_on = false;
                self.send_event(PadEventCode::IgnOff, 0);
            }
            self.hal.set_ign(false);
            return;
        }
        if self.ign_spent {
            self.hal.set_ign(false);
            return;
        }
        if !self.ign_on {
            self.ign_on = true;
            self.ign_since = now;
            self.send_event(PadEventCode::IgnOn, 0);
            self.hal.set_ign(true);
            return;
        }
        if elapsed_ms(now, self.ign_since) > self.ign_max_ds as u32 * 100 {
            self.ign_on = false;
            self.ign_spent = true;
            self.hal.set_ign(false);
            self.send_event(PadEventCode::IgnOff, 1);
            return;
        }
        self.hal.set_ign(true);
    }

    pub fn state(&self, now: u32) -> PadState {
        let o = self.outputs;
        let age = match self.last_cmd {
            Some(_) => elapsed_ms(now, self.last_cmd_ms).min(pad::AGE_NEVER as u32 - 1) as u16,
            None => pad::AGE_NEVER,
        };
        let mut flags = 0u8;
        if !o.comms_loss {
            flags |= state_flags::LINK_OK;
        }
        if self.last_cmd.is_none() {
            flags |= state_flags::LATCHED;
        }
        if o.armed {
            flags |= state_flags::ARMED;
        }
        if self.ign_on {
            flags |= state_flags::IGN_ON;
        }
        if o.comms_loss {
            flags |= state_flags::COMMS_LOSS;
        }
        if self.run_state == PadRunState::Run {
            flags |= state_flags::HELLO_OK;
        }
        if self.ign_spent {
            flags |= state_flags::IGN_SPENT;
        }
        let crc_err = (self.decoder.stats().crc_errors + self.decoder.stats().malformed)
            .wrapping_add(self.bad_cmds as u32);
        PadState {
            flags,
            out_fill: o.fill,
            out_veh: o.veh,
            out_prg: self.allow_purge && o.prg,
            out_ign: self.ign_on,
            ign_src: o.ign_src,
            age_ms: age,
            rx_count: self.rx_count,
            crc_err: crc_err as u16,
            rssi_dbm: self.hal.rssi_dbm().unwrap_or(pad::RSSI_UNKNOWN),
            pad_state: self.run_state as u8,
            uptime_s: (elapsed_ms(now, self.boot_ms) / 1000) as u16,
            wdg_ds: self.wdg_ds,
            ign_max_ds: self.ign_max_ds,
            fw_major: FIRMWARE_VERSION.0,
            fw_minor: FIRMWARE_VERSION.1,
            travel_fill: self.travel[0],
            travel_veh: self.travel[1],
            travel_prg: self.travel[2],
            allow_purge: self.allow_purge,
        }
    }

    /// The config in force, as a [`PadConfig`].
    pub fn config(&self) -> PadConfig {
        PadConfig {
            travel_fill: self.travel[0],
            travel_veh: self.travel[1],
            travel_prg: self.travel[2],
            wdg_ds: self.wdg_ds,
            ign_max_ds: self.ign_max_ds,
            allow_purge: self.allow_purge,
            cfg_seq: 0,
        }
    }

    /// GSE state snapshot of the physical outputs (answers a GSE PING).
    fn snapshot(&self, now: u32) -> StateSnapshot {
        let o = self.outputs;
        let mut f = 0u16;
        if o.fill {
            f |= flags::FILL_OPEN;
        }
        if o.veh {
            f |= flags::SRV_ON;
        }
        if self.ign_on {
            f |= flags::IGN_PWR;
        }
        if o.comms_loss {
            f |= flags::LINK_LOSS_SAFE;
        }
        StateSnapshot {
            flags: f,
            uptime_ms: elapsed_ms(now, self.boot_ms),
            ..StateSnapshot::default()
        }
    }

    fn send_state(&mut self, now: u32) {
        let state = self.state(now);
        self.send(&Message::PadState(state));
    }

    fn send_event(&mut self, code: PadEventCode, arg: u8) {
        self.send(&Message::PadEvent(PadEvent::new(code, arg)));
    }

    fn send_hello(&mut self, now: u32) {
        self.last_hello_ms = now;
        let hello = Message::Hello(Hello {
            protocol: PROTOCOL_VERSION,
            fw: [FIRMWARE_VERSION.0, FIRMWARE_VERSION.1, FIRMWARE_VERSION.2],
            role: Endpoint::Vehicle,
            uptime_ms: elapsed_ms(now, self.boot_ms),
            device_id: self.hal.device_id(),
            capabilities: capabilities::RADIO
                | capabilities::SERVOS
                | capabilities::IGNITER
                | capabilities::WATCHDOG
                | capabilities::PAD_STREAM
                | capabilities::PURGE
                | capabilities::RADIO_ESPNOW,
        });
        self.send(&hello);
    }

    fn send(&mut self, message: &Message) -> bool {
        if !self.hal.radio_ready() {
            return false;
        }
        let mut buf = [0u8; Frame::MAX_ENCODED];
        let seq = self.tx_seq;
        match encode_message(message, Endpoint::Vehicle, Endpoint::Host, seq, &mut buf) {
            Some(n) => {
                self.hal.write_radio(&buf[..n]);
                self.tx_seq = seq.wrapping_add(1);
                true
            }
            None => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockPad;
    use gse_proto::pad::ARM_CODE;

    fn boot() -> Pad<MockPad> {
        let mut pad = Pad::new(MockPad::default());
        pad.poll();
        pad.hal_mut().drain_frames();
        pad
    }

    fn cmd(fill: bool, veh: bool, prg: bool, ign: bool, armed: bool) -> PadCommand {
        PadCommand {
            arm_code: if armed { ARM_CODE } else { 0 },
            fill,
            veh,
            prg,
            ign,
            wdg_ds: 10,
            ign_max_ds: 5,
            flags: 0,
            tx_ms: 0,
            ign_wdg: false,
        }
    }

    /// Feed `c` every 50 ms for `ms` milliseconds.
    fn stream(pad: &mut Pad<MockPad>, c: PadCommand, ms: u32) {
        let mut t = 0;
        while t < ms {
            pad.hal_mut()
                .feed_frame(&Message::PadCmd(c), Endpoint::Host, Endpoint::Vehicle, 1);
            for _ in 0..5 {
                pad.hal_mut().advance(10);
                pad.poll();
            }
            t += 50;
        }
    }

    fn last_state(pad: &mut Pad<MockPad>) -> PadState {
        pad.hal_mut()
            .drain_frames()
            .iter()
            .rev()
            .find_map(|f| match f.message() {
                Some(Message::PadState(s)) => Some(s),
                _ => None,
            })
            .expect("pad state")
    }

    #[test]
    fn boots_safe_and_latched() {
        let mut pad = Pad::new(MockPad::default());
        pad.poll();
        let frames = pad.hal_mut().drain_frames();
        assert!(frames.iter().any(
            |f| matches!(f.message(), Some(Message::Hello(h)) if h.role == Endpoint::Vehicle)
        ));
        assert!(frames.iter().any(|f| matches!(
            f.message(),
            Some(Message::PadEvent(e)) if e.kind() == Some(PadEventCode::Boot)
        )));
        let st = frames
            .iter()
            .find_map(|f| match f.message() {
                Some(Message::PadState(s)) => Some(s),
                _ => None,
            })
            .unwrap();
        assert!(st.latched() && st.comms_loss() && !st.out_fill && st.out_veh && !st.out_ign);
        assert_eq!(pad.hal().servo(PadServo::Veh), 2_000);
        assert_eq!(pad.hal().servo(PadServo::Veh2), 2_000);
        assert_eq!(pad.hal().servo(PadServo::Fill), 1_000);
        assert!(!pad.hal().ign);
    }

    #[test]
    fn follows_the_stream_and_drives_both_vehicle_servos() {
        let mut pad = boot();
        stream(&mut pad, cmd(true, false, false, false, false), 200);
        assert_eq!(pad.hal().servo(PadServo::Fill), 2_000);
        assert_eq!(pad.hal().servo(PadServo::Veh), 1_000);
        assert_eq!(pad.hal().servo(PadServo::Veh2), 1_000);
        let st = last_state(&mut pad);
        assert!(st.link_ok() && st.hello_ok() && st.out_fill && !st.out_veh);
        assert!(st.rx_count >= 3);
    }

    #[test]
    fn watchdog_returns_to_safe_set() {
        let mut pad = boot();
        stream(&mut pad, cmd(true, false, false, false, false), 200);
        pad.hal_mut().drain_frames();
        for _ in 0..120 {
            pad.hal_mut().advance(10);
            pad.poll();
        }
        assert_eq!(pad.hal().servo(PadServo::Fill), 1_000);
        assert_eq!(pad.hal().servo(PadServo::Veh), 2_000);
        let frames = pad.hal_mut().drain_frames();
        assert!(frames.iter().any(|f| matches!(
            f.message(),
            Some(Message::PadEvent(e)) if e.kind() == Some(PadEventCode::CommsLoss)
        )));
    }

    #[test]
    fn purge_is_forced_closed_until_enabled() {
        let mut pad = boot();
        stream(&mut pad, cmd(false, false, true, false, false), 100);
        assert_eq!(pad.hal().servo(PadServo::Purge), 1_000);
        pad.hal_mut().feed_frame(
            &Message::PadConfig(PadConfig {
                allow_purge: true,
                cfg_seq: 4,
                ..PadConfig::default()
            }),
            Endpoint::Host,
            Endpoint::Vehicle,
            2,
        );
        stream(&mut pad, cmd(false, false, true, false, false), 100);
        assert_eq!(pad.hal().servo(PadServo::Purge), 2_000);
        let st = last_state(&mut pad);
        assert!(st.out_prg && st.allow_purge);
    }

    #[test]
    fn igniter_needs_arm_and_fires_once_per_press() {
        let mut pad = boot();
        stream(&mut pad, cmd(false, true, false, true, false), 200);
        assert!(!pad.hal().ign, "unarmed request must not fire");

        stream(&mut pad, cmd(false, true, false, true, true), 100);
        assert!(pad.hal().ign);
        // Held past the 0.5 s limit: cut and stays off.
        stream(&mut pad, cmd(false, true, false, true, true), 600);
        assert!(!pad.hal().ign);
        assert!(pad.ign_spent());
        let st = last_state(&mut pad);
        assert!(st.ign_spent() && !st.ign_on() && st.armed());
        // Release, then press again: fires again.
        stream(&mut pad, cmd(false, true, false, false, true), 100);
        assert!(!pad.ign_spent());
        stream(&mut pad, cmd(false, true, false, true, true), 100);
        assert!(pad.hal().ign);
    }

    #[test]
    fn ground_safe_commands_are_obeyed() {
        let mut pad = boot();
        stream(&mut pad, cmd(true, false, false, false, false), 100);
        pad.hal_mut().feed_frame(
            &Message::PadCmd(PadCommand::safe(10, 5)),
            Endpoint::Ground,
            Endpoint::Vehicle,
            3,
        );
        for _ in 0..5 {
            pad.hal_mut().advance(10);
            pad.poll();
        }
        assert_eq!(pad.hal().servo(PadServo::Fill), 1_000);
        assert_eq!(pad.hal().servo(PadServo::Veh), 2_000);
    }

    #[test]
    fn answers_host_ping_with_physical_outputs() {
        let mut pad = boot();
        stream(&mut pad, cmd(true, false, false, false, false), 100);
        pad.hal_mut().drain_frames();
        pad.hal_mut().feed_frame(
            &Message::Ping(gse_proto::Ping {
                nonce: 77,
                host_ms: 0,
            }),
            Endpoint::Host,
            Endpoint::Vehicle,
            4,
        );
        pad.poll();
        let pong = pad
            .hal_mut()
            .drain_frames()
            .iter()
            .find_map(|f| match f.message() {
                Some(Message::Pong(p)) => Some(p),
                _ => None,
            })
            .unwrap();
        assert_eq!(pong.nonce, 77);
        assert!(pong.state.fill_open() && !pong.state.srv_on());
    }
}
