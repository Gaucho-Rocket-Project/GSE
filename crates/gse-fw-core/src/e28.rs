//! Portable Semtech `SX1280` / `SX1281` + PA/LNA packet transceiver driver for
//! CDEBYTE **`E28-2G4M27S`** (`2.4 GHz`, `500 mW` / `+27 dBm`, `-132 dBm` RX).
//!
//! Designed strictly without heap allocation (`no_std`) or blocking infinite
//! loops so a disconnected or faulted RF module can never hang the main loop or
//! starve the hardware watchdog (`IWDG1`).
//!
//! ### Hardware & RF front-end contract (`NUCLEO-H753ZI` `SPI1` wiring)
//! * `SPI1` Mode 0 (`CPOL=0, CPHA=0`, MSB-first, `4 MHz`): `PA5` `SCK`, `PA6` `MISO`,
//!   `PA7` `MOSI`, `PD14` `NSS` (active-LOW).
//! * `PG10` `BUSY` (active-HIGH input), `PG11` `NRESET` (active-LOW output),
//!   `PF13` `DIO1` (active-HIGH IRQ input).
//! * External front-end pins (`PG12` `RX_EN` LNA, `PG13` `TX_EN` `500 mW` PA) are
//!   switched **break-before-make**: `RX_EN` and `TX_EN` are never driven `HIGH`
//!   simultaneously, and `SX1280`/`SX1281` internal output power is set to `0 dBm`
//!   (`IC_TX_POWER_REG = 18`) per the `E28-2G4M27S` datasheet so the external PA
//!   delivers linear `+27 dBm` without saturating supply current.
//!
//! ### Stream-to-packet framing
//! Higher layers (`Ground` / `Vehicle`) exchange `0x00`-delimited COBS frames of
//! at most [`Frame::MAX_ENCODED`] (`64`) bytes. [`E28Driver`] slices outbound bytes
//! on `0x00` delimiters so each GSE frame is transmitted over the air as one self-
//! contained LoRa packet with hardware LoRa header length + LoRa CRC-16 + GSE CRC-16,
//! and feeds received packets seamlessly into a bounded RX ring for `read()`.

use gse_proto::radio::e28;
use gse_proto::Frame;

use crate::hal::elapsed_ms;

const RX_RING_CAP: usize = 512;
const TX_PKT_QUEUE_SLOTS: usize = 8;

/// Host safety timeout (`ms`) if neither `DIO1` nor `IRQ_TX_DONE` clears TX state.
pub const HOST_TX_WATCHDOG_MS: u32 = 300;

/// Hardware interface to the `SX1280` / `SX1281` SPI bus and `E28-2G4M27S` pins.
pub trait Sx128xBus {
    /// Drive `NSS` (`false` = selected / active-LOW, `true` = idle HIGH).
    fn set_nss(&mut self, high: bool);
    /// Drive `NRESET` (`false` = asserted reset LOW, `true` = running HIGH).
    fn set_nreset(&mut self, high: bool);
    /// Break-before-make front-end control for onboard LNA (`RX_EN`) and `500 mW` PA (`TX_EN`).
    fn set_rf_switch(&mut self, rx_en: bool, tx_en: bool);
    /// Read `BUSY` (`HIGH` while SX128x internal state machine is processing a command).
    fn is_busy(&self) -> bool;
    /// Read `DIO1` (`HIGH` when a masked IRQ bit is asserted).
    fn dio1_high(&self) -> bool;
    /// Full-duplex Mode-0 SPI byte transfer (`tx.len() == rx.len()`).
    /// Returns `false` if the hardware SPI peripheral timed out.
    fn spi_transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool;
    /// Short bounded spin/delay in microseconds used only during `init()` reset pulse.
    fn delay_us(&mut self, us: u32);
}

#[derive(Clone, Copy)]
struct PacketSlot {
    len: u8,
    bytes: [u8; Frame::MAX_ENCODED],
}

impl PacketSlot {
    const fn empty() -> Self {
        PacketSlot {
            len: 0,
            bytes: [0u8; Frame::MAX_ENCODED],
        }
    }
}

/// Runtime telemetry counters maintained by [`E28Driver`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct E28Stats {
    pub tx_packets: u32,
    pub rx_packets: u32,
    pub hw_crc_errors: u16,
    pub tx_timeouts: u16,
    pub last_rssi_dbm: i8,
}

/// Non-blocking `E28-2G4M27S` (`SX1280` / `SX1281`) LoRa packet transceiver driver.
pub struct E28Driver<B: Sx128xBus> {
    bus: B,
    present: bool,
    tx_active: bool,
    tx_started_ms: u32,
    stage: [u8; Frame::MAX_ENCODED],
    stage_len: usize,
    tx_queue: [PacketSlot; TX_PKT_QUEUE_SLOTS],
    tx_head: usize,
    tx_tail: usize,
    rx_ring: [u8; RX_RING_CAP],
    rx_head: usize,
    rx_tail: usize,
    stats: E28Stats,
}

impl<B: Sx128xBus> E28Driver<B> {
    pub const fn new(bus: B) -> Self {
        E28Driver {
            bus,
            present: false,
            tx_active: false,
            tx_started_ms: 0,
            stage: [0u8; Frame::MAX_ENCODED],
            stage_len: 0,
            tx_queue: [PacketSlot::empty(); TX_PKT_QUEUE_SLOTS],
            tx_head: 0,
            tx_tail: 0,
            rx_ring: [0u8; RX_RING_CAP],
            rx_head: 0,
            rx_tail: 0,
            stats: E28Stats {
                tx_packets: 0,
                rx_packets: 0,
                hw_crc_errors: 0,
                tx_timeouts: 0,
                last_rssi_dbm: -127,
            },
        }
    }

    pub fn bus(&self) -> &B {
        &self.bus
    }

    pub fn bus_mut(&mut self) -> &mut B {
        &mut self.bus
    }

    pub fn is_present(&self) -> bool {
        self.present
    }

    pub fn stats(&self) -> E28Stats {
        self.stats
    }

    /// True while packets remain queued or the PA is actively transmitting on air.
    pub fn tx_busy(&self) -> bool {
        self.tx_active || self.tx_head != self.tx_tail || self.stage_len > 0
    }

    /// Put the RF front-end into safe idle (`RX_EN=0, TX_EN=0, NSS=1`) and standby.
    pub fn shutdown(&mut self) {
        self.bus.set_rf_switch(false, false);
        self.bus.set_nss(true);
        if self.present {
            let _ = self.cmd(&[e28::CMD_SET_STANDBY, e28::STDBY_RC]);
        }
        self.tx_active = false;
        self.stage_len = 0;
        self.tx_head = 0;
        self.tx_tail = 0;
    }

    /// Reset, calibrate, configure `2424 MHz` LoRa (`SF8 / BW 406.25 kHz / CR 4/5`,
    /// `+27 dBm` PA / high-sensitivity LNA), verify real `SX1280`/`SX1281` silicon
    /// presence over SPI (`REG_LR_PAYLOAD_LENGTH == 0x40`), and enter continuous RX.
    pub fn init(&mut self) -> bool {
        self.present = false;
        self.tx_active = false;
        self.stage_len = 0;
        self.tx_head = 0;
        self.tx_tail = 0;
        self.rx_head = 0;
        self.rx_tail = 0;

        // 1. De-energise PA/LNA and pulse NRESET low for 2 ms, then settle 5 ms.
        self.bus.set_rf_switch(false, false);
        self.bus.set_nss(true);
        self.bus.set_nreset(false);
        self.bus.delay_us(2_000);
        self.bus.set_nreset(true);
        self.bus.delay_us(5_000);

        if !self.wait_ready() {
            return false;
        }

        // 2. Standby RC -> DC-DC regulator -> full block calibration -> LoRa mode.
        if !self.cmd(&[e28::CMD_SET_STANDBY, e28::STDBY_RC]) {
            return false;
        }
        if !self.cmd(&[e28::CMD_SET_REGULATOR_MODE, e28::REGULATOR_DCDC]) {
            return false;
        }
        if !self.cmd(&[e28::CMD_CALIBRATE, e28::CALIBRATE_ALL]) {
            return false;
        }
        if !self.wait_ready() {
            return false;
        }
        if !self.cmd(&[e28::CMD_SET_PACKET_TYPE, e28::PACKET_TYPE_LORA]) {
            return false;
        }

        // 3. Carrier frequency = 2424.000 MHz (`[0xBA, 0x76, 0x27]`).
        let pll = e28::DEFAULT_RF_PLL;
        if !self.cmd(&[e28::CMD_SET_RF_FREQUENCY, pll[0], pll[1], pll[2]]) {
            return false;
        }

        // 4. Split 256-byte buffer cleanly between TX (0x00) and RX (0x80).
        if !self.cmd(&[
            e28::CMD_SET_BUFFER_BASE_ADDRESS,
            e28::TX_BASE_ADDR,
            e28::RX_BASE_ADDR,
        ]) {
            return false;
        }

        // 5. LoRa modulation: SF8, BW 406.25 kHz, CR 4/5 + SX1280 DS §14.4.1 trims.
        if !self.cmd(&[
            e28::CMD_SET_MODULATION_PARAMS,
            e28::LORA_SF8,
            e28::LORA_BW_0400,
            e28::LORA_CR_4_5,
        ]) {
            return false;
        }
        if !self.write_reg(
            e28::REG_LR_SF_ADDITIONAL,
            e28::sf_additional_reg(e28::LORA_SF8),
        ) {
            return false;
        }
        if !self.write_reg(e28::REG_LR_FREQ_ERR_COMP, 0x01) {
            return false;
        }
        // Enable High-Sensitivity LNA regime (`REG_LNA_REGIME |= 0xC0`, SX1280 DS §4.2).
        if let Some(lna) = self.read_reg(e28::REG_LNA_REGIME) {
            let _ = self.write_reg(e28::REG_LNA_REGIME, lna | 0xC0);
        }

        // 6. Explicit-header variable-length packets up to `MAX_PACKET_LEN` (`0x40` = 64 B),
        //    hardware CRC-16 ON, standard IQ polarity.
        if !self.set_lora_packet_len(e28::MAX_PACKET_LEN) {
            return false;
        }

        // 7. Silicon presence check: `SetPacketParams` writes `MAX_PACKET_LEN` (`0x40`)
        //    into `REG_LR_PAYLOAD_LENGTH` (`0x0901`). An unpopulated header (`MISO` stuck
        //    at `0x00` or `0xFF`) reads `0x00` or `0xFF`, never `0x40`.
        if self.read_reg(e28::REG_LR_PAYLOAD_LENGTH) != Some(e28::MAX_PACKET_LEN) {
            return false;
        }

        // 8. Output power: SX128x internal `0 dBm` (`18`) + `20 us` ramp -> onboard PA `+27 dBm`.
        if !self.cmd(&[
            e28::CMD_SET_TX_PARAMS,
            e28::IC_TX_POWER_REG,
            e28::RADIO_RAMP_20_US,
        ]) {
            return false;
        }

        // 9. Route `TxDone | RxDone | HeaderError | CrcError | RxTxTimeout` to `DIO1`.
        let mask = e28::IRQ_DEFAULT_MASK.to_be_bytes();
        if !self.cmd(&[
            e28::CMD_SET_DIO_IRQ_PARAMS,
            mask[0],
            mask[1],
            mask[0],
            mask[1],
            0x00,
            0x00,
            0x00,
            0x00,
        ]) {
            return false;
        }

        self.present = self.enter_rx();
        self.present
    }

    /// True when the packet queue can accept at least one more complete frame.
    pub fn ready(&self) -> bool {
        self.present && self.tx_queue_len() < TX_PKT_QUEUE_SLOTS - 1
    }

    /// Queue COBS-framed stream bytes (`0x00` ... `0x00`) for packet transmission.
    pub fn write(&mut self, data: &[u8]) -> bool {
        if !self.present {
            return false;
        }
        for &b in data {
            if b == 0 {
                if self.stage_len > 0 {
                    // Append closing 0x00 delimiter and commit one atomic RF packet.
                    if self.stage_len < Frame::MAX_ENCODED {
                        self.stage[self.stage_len] = 0;
                        self.stage_len += 1;
                    }
                    let len = self.stage_len;
                    self.stage_len = 0;
                    let pkt = self.stage;
                    if !self.enqueue_packet(&pkt[..len]) {
                        return false;
                    }
                }
            } else {
                if self.stage_len == 0 {
                    // Include leading 0x00 frame delimiter so stream decoders see clean boundaries.
                    self.stage[0] = 0;
                    self.stage_len = 1;
                }
                if self.stage_len < Frame::MAX_ENCODED {
                    self.stage[self.stage_len] = b;
                    self.stage_len += 1;
                }
            }
        }
        true
    }

    /// Drain received frame bytes into `out`.
    pub fn read(&mut self, out: &mut [u8]) -> usize {
        let mut n = 0;
        while n < out.len() && self.rx_head != self.rx_tail {
            out[n] = self.rx_ring[self.rx_tail % RX_RING_CAP];
            self.rx_tail = self.rx_tail.wrapping_add(1);
            n += 1;
        }
        n
    }

    /// Service `SX1280`/`SX1281` IRQs, complete TX->RX transitions, drain incoming
    /// packets from RX FIFO, and kick off the next queued outbound packet.
    pub fn pump(&mut self, now_ms: u32) {
        if !self.present {
            return;
        }

        // Poll IRQ status whenever DIO1 asserts or while waiting on TX completion.
        if self.bus.dio1_high() || self.tx_active {
            if let Some(irq) = self.get_irq_status() {
                if irq != 0 {
                    let _ = self.clear_irq_status(irq);

                    if irq & (e28::IRQ_CRC_ERROR | e28::IRQ_HEADER_ERROR) != 0 {
                        self.stats.hw_crc_errors = self.stats.hw_crc_errors.wrapping_add(1);
                    } else if irq & e28::IRQ_RX_DONE != 0 {
                        self.harvest_rx_packet();
                    }

                    if irq & e28::IRQ_TX_DONE != 0 {
                        self.tx_active = false;
                        self.stats.tx_packets = self.stats.tx_packets.wrapping_add(1);
                        let _ = self.enter_rx();
                    } else if self.tx_active && (irq & e28::IRQ_RX_TX_TIMEOUT != 0) {
                        self.tx_active = false;
                        self.stats.tx_timeouts = self.stats.tx_timeouts.wrapping_add(1);
                        let _ = self.enter_rx();
                    }
                }
            }
        }

        // Software safety net so a missed IRQ edge can never leave the 500 mW PA keyed.
        if self.tx_active && elapsed_ms(now_ms, self.tx_started_ms) >= HOST_TX_WATCHDOG_MS {
            self.tx_active = false;
            self.stats.tx_timeouts = self.stats.tx_timeouts.wrapping_add(1);
            let _ = self.enter_rx();
        }

        if !self.tx_active && self.tx_head != self.tx_tail {
            let slot = self.tx_queue[self.tx_tail % TX_PKT_QUEUE_SLOTS];
            self.tx_tail = self.tx_tail.wrapping_add(1);
            if self.transmit_slot(&slot, now_ms) {
                self.tx_active = true;
                self.tx_started_ms = now_ms;
            } else {
                let _ = self.enter_rx();
            }
        }
    }

    fn tx_queue_len(&self) -> usize {
        self.tx_head.wrapping_sub(self.tx_tail)
    }

    fn rx_free(&self) -> usize {
        RX_RING_CAP - self.rx_head.wrapping_sub(self.rx_tail)
    }

    fn enqueue_packet(&mut self, bytes: &[u8]) -> bool {
        if bytes.is_empty()
            || bytes.len() > Frame::MAX_ENCODED
            || self.tx_queue_len() >= TX_PKT_QUEUE_SLOTS
        {
            return false;
        }
        let idx = self.tx_head % TX_PKT_QUEUE_SLOTS;
        self.tx_queue[idx].len = bytes.len() as u8;
        self.tx_queue[idx].bytes[..bytes.len()].copy_from_slice(bytes);
        self.tx_head = self.tx_head.wrapping_add(1);
        true
    }

    fn set_lora_packet_len(&mut self, len: u8) -> bool {
        self.cmd(&[
            e28::CMD_SET_PACKET_PARAMS,
            e28::LORA_PREAMBLE_SYMBOLS,
            e28::LORA_PACKET_EXPLICIT,
            len,
            e28::LORA_CRC_ON,
            e28::LORA_IQ_NORMAL,
            0x00,
            0x00,
        ])
    }

    fn enter_rx(&mut self) -> bool {
        // Break-before-make: ensure PA is OFF before enabling LNA.
        self.bus.set_rf_switch(false, false);
        let _ = self.set_lora_packet_len(e28::MAX_PACKET_LEN);
        let _ = self.clear_irq_status(0xFFFF);
        self.bus.set_rf_switch(true, false);
        let steps = e28::RX_CONTINUOUS_STEPS.to_be_bytes();
        self.cmd(&[e28::CMD_SET_RX, e28::PERIOD_BASE_1_MS, steps[0], steps[1]])
    }

    fn transmit_slot(&mut self, slot: &PacketSlot, _now_ms: u32) -> bool {
        let len = slot.len as usize;
        if len == 0 || len > Frame::MAX_ENCODED {
            return false;
        }
        // Standby + break-before-make switch from LNA to 500 mW PA.
        self.bus.set_rf_switch(false, false);
        if !self.cmd(&[e28::CMD_SET_STANDBY, e28::STDBY_RC]) {
            return false;
        }
        if !self.set_lora_packet_len(slot.len) {
            return false;
        }
        if !self.write_buffer(e28::TX_BASE_ADDR, &slot.bytes[..len]) {
            return false;
        }
        let _ = self.clear_irq_status(0xFFFF);
        self.bus.set_rf_switch(false, true);
        let timeout = e28::TX_TIMEOUT_MS.to_be_bytes();
        self.cmd(&[
            e28::CMD_SET_TX,
            e28::PERIOD_BASE_1_MS,
            timeout[0],
            timeout[1],
        ])
    }

    fn harvest_rx_packet(&mut self) {
        let (len, offset) = match self.get_rx_buffer_status() {
            Some(v) => v,
            None => return,
        };
        let n = (len as usize).min(Frame::MAX_ENCODED);
        if n == 0 || self.rx_free() < n + 1 {
            return;
        }
        let mut pkt = [0u8; Frame::MAX_ENCODED];
        if !self.read_buffer(offset, &mut pkt[..n]) {
            return;
        }
        if let Some(rssi) = self.get_packet_rssi_dbm() {
            self.stats.last_rssi_dbm = rssi;
        }
        for &b in &pkt[..n] {
            self.rx_ring[self.rx_head % RX_RING_CAP] = b;
            self.rx_head = self.rx_head.wrapping_add(1);
        }
        // Guarantee trailing frame delimiter even if sender omitted leading/trailing 0x00.
        if pkt[n - 1] != 0 && self.rx_free() > 0 {
            self.rx_ring[self.rx_head % RX_RING_CAP] = 0;
            self.rx_head = self.rx_head.wrapping_add(1);
        }
        self.stats.rx_packets = self.stats.rx_packets.wrapping_add(1);
    }

    fn wait_ready(&mut self) -> bool {
        for _ in 0..2_000 {
            if !self.bus.is_busy() {
                return true;
            }
            self.bus.delay_us(10);
        }
        false
    }

    fn cmd(&mut self, tx: &[u8]) -> bool {
        if tx.len() > 16 || !self.wait_ready() {
            return false;
        }
        let mut rx = [0u8; 16];
        self.bus.set_nss(false);
        let ok = self.bus.spi_transfer(tx, &mut rx[..tx.len()]);
        self.bus.set_nss(true);
        ok
    }

    fn write_reg(&mut self, addr: u16, val: u8) -> bool {
        let hi = (addr >> 8) as u8;
        let lo = (addr & 0xFF) as u8;
        self.cmd(&[e28::CMD_WRITE_REGISTER, hi, lo, val])
    }

    fn read_reg(&mut self, addr: u16) -> Option<u8> {
        if !self.wait_ready() {
            return None;
        }
        let tx = [
            e28::CMD_READ_REGISTER,
            (addr >> 8) as u8,
            (addr & 0xFF) as u8,
            0x00,
            0x00,
        ];
        let mut rx = [0u8; 5];
        self.bus.set_nss(false);
        let ok = self.bus.spi_transfer(&tx, &mut rx);
        self.bus.set_nss(true);
        if ok {
            Some(rx[4])
        } else {
            None
        }
    }

    fn write_buffer(&mut self, offset: u8, payload: &[u8]) -> bool {
        if payload.len() > Frame::MAX_ENCODED || !self.wait_ready() {
            return false;
        }
        let mut tx = [0u8; 2 + Frame::MAX_ENCODED];
        let mut rx = [0u8; 2 + Frame::MAX_ENCODED];
        tx[0] = e28::CMD_WRITE_BUFFER;
        tx[1] = offset;
        tx[2..2 + payload.len()].copy_from_slice(payload);
        self.bus.set_nss(false);
        let ok = self
            .bus
            .spi_transfer(&tx[..2 + payload.len()], &mut rx[..2 + payload.len()]);
        self.bus.set_nss(true);
        ok
    }

    fn read_buffer(&mut self, offset: u8, out: &mut [u8]) -> bool {
        if out.len() > Frame::MAX_ENCODED || !self.wait_ready() {
            return false;
        }
        let mut tx = [0u8; 3 + Frame::MAX_ENCODED];
        let mut rx = [0u8; 3 + Frame::MAX_ENCODED];
        tx[0] = e28::CMD_READ_BUFFER;
        tx[1] = offset;
        tx[2] = 0x00;
        self.bus.set_nss(false);
        let ok = self
            .bus
            .spi_transfer(&tx[..3 + out.len()], &mut rx[..3 + out.len()]);
        self.bus.set_nss(true);
        if ok {
            out.copy_from_slice(&rx[3..3 + out.len()]);
        }
        ok
    }

    fn get_irq_status(&mut self) -> Option<u16> {
        if !self.wait_ready() {
            return None;
        }
        let tx = [e28::CMD_GET_IRQ_STATUS, 0x00, 0x00, 0x00];
        let mut rx = [0u8; 4];
        self.bus.set_nss(false);
        let ok = self.bus.spi_transfer(&tx, &mut rx);
        self.bus.set_nss(true);
        if ok {
            Some(u16::from_be_bytes([rx[2], rx[3]]))
        } else {
            None
        }
    }

    fn clear_irq_status(&mut self, mask: u16) -> bool {
        let b = mask.to_be_bytes();
        self.cmd(&[e28::CMD_CLR_IRQ_STATUS, b[0], b[1]])
    }

    fn get_rx_buffer_status(&mut self) -> Option<(u8, u8)> {
        if !self.wait_ready() {
            return None;
        }
        let tx = [e28::CMD_GET_RX_BUFFER_STATUS, 0x00, 0x00, 0x00];
        let mut rx = [0u8; 4];
        self.bus.set_nss(false);
        let ok = self.bus.spi_transfer(&tx, &mut rx);
        self.bus.set_nss(true);
        if ok {
            Some((rx[2], rx[3]))
        } else {
            None
        }
    }

    fn get_packet_rssi_dbm(&mut self) -> Option<i8> {
        if !self.wait_ready() {
            return None;
        }
        let tx = [
            e28::CMD_GET_PACKET_STATUS,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ];
        let mut rx = [0u8; 7];
        self.bus.set_nss(false);
        let ok = self.bus.spi_transfer(&tx, &mut rx);
        self.bus.set_nss(true);
        if ok {
            // SX1280 DS §11.8.2: LoRa RSSI_packet = -rssiSync / 2 (dBm).
            let dbm = -((rx[2] as i16) / 2);
            Some(dbm.clamp(-127, 0) as i8)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gse_proto::{encode_message, Command, Control, Endpoint, FrameDecoder, Message};

    /// Cycle-accurate register + FIFO model of a real Semtech `SX1280`/`SX1281`
    /// wired to an `E28-2G4M27S` PA/LNA front-end.
    struct SimSx1281 {
        unplugged: bool,
        nss_high: bool,
        nreset_high: bool,
        rx_en: bool,
        tx_en: bool,
        both_high_ever: bool,
        regs: [u8; 0x0A00],
        fifo: [u8; 256],
        tx_base: u8,
        rx_base: u8,
        pkt_len: u8,
        rx_payload_len: u8,
        irq: u16,
        dio1_mask: u16,
        in_rx: bool,
        air_tx: Option<Vec<u8>>,
        rf_pll: [u8; 3],
        tx_power_reg: u8,
    }

    impl SimSx1281 {
        fn new(unplugged: bool) -> Self {
            SimSx1281 {
                unplugged,
                nss_high: true,
                nreset_high: true,
                rx_en: false,
                tx_en: false,
                both_high_ever: false,
                regs: [0u8; 0x0A00],
                fifo: [0u8; 256],
                tx_base: 0,
                rx_base: 0,
                pkt_len: 0,
                rx_payload_len: 0,
                irq: 0,
                dio1_mask: 0,
                in_rx: false,
                air_tx: None,
                rf_pll: [0; 3],
                tx_power_reg: 0,
            }
        }

        fn deliver_air_packet(&mut self, pkt: &[u8]) {
            assert!(
                self.in_rx && self.rx_en && !self.tx_en,
                "receiver must be in LNA RX mode"
            );
            let base = self.rx_base as usize;
            self.fifo[base..base + pkt.len()].copy_from_slice(pkt);
            self.rx_payload_len = pkt.len() as u8;
            self.irq |= e28::IRQ_RX_DONE;
        }
    }

    impl Sx128xBus for SimSx1281 {
        fn set_nss(&mut self, high: bool) {
            self.nss_high = high;
        }

        fn set_nreset(&mut self, high: bool) {
            if !high {
                self.regs.fill(0);
                self.irq = 0;
                self.in_rx = false;
            }
            self.nreset_high = high;
        }

        fn set_rf_switch(&mut self, rx_en: bool, tx_en: bool) {
            if rx_en && tx_en {
                self.both_high_ever = true;
            }
            self.rx_en = rx_en;
            self.tx_en = tx_en;
        }

        fn is_busy(&self) -> bool {
            false
        }

        fn dio1_high(&self) -> bool {
            !self.unplugged && (self.irq & self.dio1_mask) != 0
        }

        fn spi_transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
            rx.fill(if self.unplugged { 0xFF } else { 0xA2 });
            if self.unplugged || tx.is_empty() {
                return true;
            }
            match tx[0] {
                e28::CMD_SET_RF_FREQUENCY => {
                    self.rf_pll = [tx[1], tx[2], tx[3]];
                }
                e28::CMD_SET_BUFFER_BASE_ADDRESS => {
                    self.tx_base = tx[1];
                    self.rx_base = tx[2];
                }
                e28::CMD_WRITE_REGISTER => {
                    let addr = u16::from_be_bytes([tx[1], tx[2]]) as usize;
                    self.regs[addr] = tx[3];
                }
                e28::CMD_READ_REGISTER => {
                    let addr = u16::from_be_bytes([tx[1], tx[2]]) as usize;
                    rx[4] = self.regs[addr];
                }
                e28::CMD_SET_PACKET_PARAMS => {
                    self.pkt_len = tx[3];
                    self.regs[e28::REG_LR_PAYLOAD_LENGTH as usize] = tx[3];
                }
                e28::CMD_SET_TX_PARAMS => {
                    self.tx_power_reg = tx[1];
                }
                e28::CMD_SET_DIO_IRQ_PARAMS => {
                    self.dio1_mask = u16::from_be_bytes([tx[3], tx[4]]);
                }
                e28::CMD_WRITE_BUFFER => {
                    let off = tx[1] as usize;
                    let data = &tx[2..];
                    self.fifo[off..off + data.len()].copy_from_slice(data);
                }
                e28::CMD_READ_BUFFER => {
                    let off = tx[1] as usize;
                    let n = rx.len() - 3;
                    rx[3..].copy_from_slice(&self.fifo[off..off + n]);
                }
                e28::CMD_SET_RX => {
                    self.in_rx = true;
                }
                e28::CMD_SET_TX => {
                    self.in_rx = false;
                    let start = self.tx_base as usize;
                    let end = start + self.pkt_len as usize;
                    self.air_tx = Some(self.fifo[start..end].to_vec());
                    self.irq |= e28::IRQ_TX_DONE;
                }
                e28::CMD_GET_IRQ_STATUS => {
                    let b = self.irq.to_be_bytes();
                    rx[2] = b[0];
                    rx[3] = b[1];
                }
                e28::CMD_CLR_IRQ_STATUS => {
                    let mask = u16::from_be_bytes([tx[1], tx[2]]);
                    self.irq &= !mask;
                }
                e28::CMD_GET_RX_BUFFER_STATUS => {
                    rx[2] = self.rx_payload_len;
                    rx[3] = self.rx_base;
                }
                e28::CMD_GET_PACKET_STATUS => {
                    rx[2] = 110; // -55 dBm
                }
                _ => {}
            }
            true
        }

        fn delay_us(&mut self, _us: u32) {}
    }

    #[test]
    fn initializes_registers_and_exchanges_frames_over_simulated_rf() {
        let mut ground_e28 = E28Driver::new(SimSx1281::new(false));
        let mut vehicle_e28 = E28Driver::new(SimSx1281::new(false));
        assert!(ground_e28.init());
        assert!(vehicle_e28.init());

        // Verify exact datasheet register programming and break-before-make PA/LNA invariant.
        assert_eq!(ground_e28.bus().rf_pll, e28::DEFAULT_RF_PLL);
        assert_eq!(ground_e28.bus().tx_power_reg, e28::IC_TX_POWER_REG);
        assert_eq!(
            ground_e28.bus().regs[e28::REG_LR_SF_ADDITIONAL as usize],
            0x37
        );
        assert_eq!(
            ground_e28.bus().regs[e28::REG_LR_FREQ_ERR_COMP as usize],
            0x01
        );
        assert!(!ground_e28.bus().both_high_ever);
        assert!(ground_e28.bus().rx_en && !ground_e28.bus().tx_en);

        // Encode a real GSE Command frame and transmit Ground -> Vehicle over E28 SPI.
        let cmd = Message::Command(Command {
            cmd_id: 42,
            control: Control::Fill,
            action: 1,
            arg_s: 30,
            flags: 0,
        });
        let mut wire = [0u8; Frame::MAX_ENCODED];
        let n = encode_message(&cmd, Endpoint::Host, Endpoint::Vehicle, 7, &mut wire).unwrap();
        assert!(ground_e28.write(&wire[..n]));
        assert!(ground_e28.tx_busy());

        ground_e28.pump(100);
        let air_pkt = ground_e28
            .bus_mut()
            .air_tx
            .take()
            .expect("LoRa packet transmitted");
        // Complete TxDone -> automatic return to continuous RX with PA off, LNA on.
        ground_e28.pump(105);
        assert!(!ground_e28.tx_busy());
        assert!(!ground_e28.bus().both_high_ever);
        assert!(ground_e28.bus().rx_en && !ground_e28.bus().tx_en);
        assert_eq!(ground_e28.stats().tx_packets, 1);

        // Deliver over the air to Vehicle E28 and decode frame intact.
        vehicle_e28.bus_mut().deliver_air_packet(&air_pkt);
        vehicle_e28.pump(106);
        assert_eq!(vehicle_e28.stats().rx_packets, 1);
        assert_eq!(vehicle_e28.stats().last_rssi_dbm, -55);

        let mut rx_buf = [0u8; 128];
        let got = vehicle_e28.read(&mut rx_buf);
        let mut dec = FrameDecoder::new();
        let mut decoded = None;
        for &b in &rx_buf[..got] {
            if let Some(f) = dec.push(b) {
                decoded = Some(f);
            }
        }
        assert_eq!(decoded.expect("decoded frame").message(), Some(cmd));
    }

    #[test]
    fn detects_missing_hardware_cleanly_without_hanging() {
        let mut missing = E28Driver::new(SimSx1281::new(true));
        assert!(!missing.init());
        assert!(!missing.is_present());
        assert!(!missing.ready());
    }
}
