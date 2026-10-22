//! DX-SMART `DX-LR42-24T12D` (Semtech SX1281, 2.4 GHz) radio constants and helpers.
//!
//! Parameters here match the manufacturer's `DX-LR42-24T12D` Hardware Datasheet V2.0
//! and `DX-LR41/LR42` UART AT Application Guide V2.0:
//!
//! * Factory UART configuration: `9600 bps, 8N1` (`AT+BAUD2`, `AT+PARI0`), `3.3 V` TTL
//! * Factory operating mode: transparent (`AT+MODE0`), continuous RX (`AT+SLEEP2`),
//!   software AT switching (`AT+SWITCH0`)
//! * Factory RF configuration: `AT+LEVEL3` (`SF8, BW 406 kHz, CR 4/5`, `10 kbps` air rate,
//!   `2.2 km` open-air line-of-sight), `AT+CHANNEL0C` (`2424 MHz`), `AT+POWE13` (`+13 dBm`),
//!   `AT+PACKET3` (`230 B` max LoRa sub-packet), `AT+DRSSI0`
//! * Supply voltage: `VCC = 3.3–5.5 V` (`5.0 V` typical recommended for full `+13 dBm`);
//!   UART / `M0` / `M1` / `AUX` logic pins are strictly `3.3 V` CMOS (`3.6 V` max).

use crate::frame::{CRC_LEN, HEADER_LEN, MAX_ENCODED_LEN};

/// Installed 2.4 GHz SX1281 UART transceiver part number.
pub const MODULE_PART: &str = "DX-LR42-24T12D";

/// Minimum possible wire length of a valid GSE frame (`0x00` + COBS(header + crc) + `0x00`).
pub const MIN_ENCODED_FRAME_LEN: usize = HEADER_LEN + CRC_LEN + 3;

/// Maximum bytes in a single LoRa packet on the `DX-LR42-24T12D` (`AT+PACKET3`, default).
pub const LORA_MAX_PACKET_BYTES: usize = 230;

/// Fixed sub-packet size in `LEVEL 8` FLRC mode (`1.04 Mbps`).
pub const FLRC_MAX_PACKET_BYTES: usize = 127;

/// Minimum payload required by `DX-LR42-24T12D` in `LEVEL 8` FLRC mode (`>= 6 B`).
pub const FLRC_MIN_PACKET_BYTES: usize = 6;

// Compile-time proof that every valid GSE frame fits in a single air packet across
// all 9 `DX-LR42-24T12D` modulation levels (LoRa `0..=7` and FLRC `8`).
const _: () = {
    assert!(MIN_ENCODED_FRAME_LEN >= FLRC_MIN_PACKET_BYTES);
    assert!(MAX_ENCODED_LEN <= FLRC_MAX_PACKET_BYTES);
    assert!(FLRC_MAX_PACKET_BYTES <= LORA_MAX_PACKET_BYTES);
};

/// Factory default RF channel (`0x0C` = `2424 MHz`).
pub const DEFAULT_CHANNEL: u8 = 0x0C;

/// Highest supported RF channel (`0x32` = `50` decimal = `2500 MHz`).
pub const MAX_CHANNEL: u8 = 0x32;

/// Base frequency for channel `0x00`, in MHz (`2 MHz` step per channel).
pub const BASE_FREQ_MHZ: u16 = 2_400;

/// Factory default and maximum RF output power, in dBm (`+13 dBm` at `VCC >= 5.0 V`).
pub const DEFAULT_POWER_DBM: u8 = 13;
pub const MAX_POWER_DBM: u8 = 13;

/// Factory default `AT+KEY` network encryption key (`AT+OPENKEY1`).
pub const DEFAULT_KEY: u16 = 12_345;

/// In-band sequence (without `\r\n`) that toggles `DX-LR42-24T12D` AT command mode
/// when hardware pin switching is disabled (`AT+SWITCH0`).
pub const AT_TOGGLE_SEQ: &[u8; 3] = b"+++";

/// Upper bound (`ms`) that firmware waits on a high `AUX` pin before bypassing flow control,
/// ensuring a miswired or stuck-high `AUX` line can never starve safety commands.
pub const AUX_MAX_WAIT_MS: u32 = 250;

/// Supported `AT+BAUD` UART speed codes (`0..=6`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum DxLr42Baud {
    Baud2400 = 0,
    Baud4800 = 1,
    #[default]
    Baud9600 = 2,
    Baud19200 = 3,
    Baud38400 = 4,
    Baud57600 = 5,
    Baud115200 = 6,
}

impl DxLr42Baud {
    pub const fn code(self) -> u8 {
        self as u8
    }

    pub const fn bps(self) -> u32 {
        match self {
            DxLr42Baud::Baud2400 => 2_400,
            DxLr42Baud::Baud4800 => 4_800,
            DxLr42Baud::Baud9600 => 9_600,
            DxLr42Baud::Baud19200 => 19_200,
            DxLr42Baud::Baud38400 => 38_400,
            DxLr42Baud::Baud57600 => 57_600,
            DxLr42Baud::Baud115200 => 115_200,
        }
    }

    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(DxLr42Baud::Baud2400),
            1 => Some(DxLr42Baud::Baud4800),
            2 => Some(DxLr42Baud::Baud9600),
            3 => Some(DxLr42Baud::Baud19200),
            4 => Some(DxLr42Baud::Baud38400),
            5 => Some(DxLr42Baud::Baud57600),
            6 => Some(DxLr42Baud::Baud115200),
            _ => None,
        }
    }

    pub const fn from_bps(bps: u32) -> Option<Self> {
        match bps {
            2_400 => Some(DxLr42Baud::Baud2400),
            4_800 => Some(DxLr42Baud::Baud4800),
            9_600 => Some(DxLr42Baud::Baud9600),
            19_200 => Some(DxLr42Baud::Baud19200),
            38_400 => Some(DxLr42Baud::Baud38400),
            57_600 => Some(DxLr42Baud::Baud57600),
            115_200 => Some(DxLr42Baud::Baud115200),
            _ => None,
        }
    }
}

/// Supported `AT+LEVEL` over-the-air rate presets (`0..=7` LoRa, `8` FLRC).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum DxLr42Level {
    /// LoRa `SF12, BW 203 kHz, CR 4/5` — `500 bps`
    Level0 = 0,
    /// LoRa `SF11, BW 406 kHz, CR 4/8` — `1 kbps`, `3.7 km` open-air range
    Level1 = 1,
    /// LoRa `SF11, BW 1625 kHz, CR 4/7` — `5 kbps`
    Level2 = 2,
    /// LoRa `SF8, BW 406 kHz, CR 4/5` — `10 kbps`, `2.2 km` open-air range (factory default)
    #[default]
    Level3 = 3,
    /// LoRa `SF6, BW 406 kHz, CR 4/8` — `20 kbps`
    Level4 = 4,
    /// LoRa `SF6, BW 806 kHz, CR 4/6` — `50 kbps`
    Level5 = 5,
    /// LoRa `SF6, BW 1625 kHz, CR 4/6` — `100 kbps`
    Level6 = 6,
    /// LoRa `SF5, BW 1625 kHz, CR 4/7` — `150 kbps`
    Level7 = 7,
    /// FLRC `BW 1.2 MHz, CR 1` — `1.04 Mbps` (`127 B` max packet, `OPENKEY` disabled)
    Level8 = 8,
}

impl DxLr42Level {
    pub const fn code(self) -> u8 {
        self as u8
    }

    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(DxLr42Level::Level0),
            1 => Some(DxLr42Level::Level1),
            2 => Some(DxLr42Level::Level2),
            3 => Some(DxLr42Level::Level3),
            4 => Some(DxLr42Level::Level4),
            5 => Some(DxLr42Level::Level5),
            6 => Some(DxLr42Level::Level6),
            7 => Some(DxLr42Level::Level7),
            8 => Some(DxLr42Level::Level8),
            _ => None,
        }
    }

    /// Nominal air bit-rate in bits/second per `DX-LR42-24T12D` manual section 5.3.2.
    pub const fn air_bps(self) -> u32 {
        match self {
            DxLr42Level::Level0 => 500,
            DxLr42Level::Level1 => 1_000,
            DxLr42Level::Level2 => 5_000,
            DxLr42Level::Level3 => 10_000,
            DxLr42Level::Level4 => 20_000,
            DxLr42Level::Level5 => 50_000,
            DxLr42Level::Level6 => 100_000,
            DxLr42Level::Level7 => 150_000,
            DxLr42Level::Level8 => 1_040_000,
        }
    }

    /// Maximum single over-the-air packet payload for this modulation level.
    pub const fn max_packet_bytes(self) -> usize {
        match self {
            DxLr42Level::Level8 => FLRC_MAX_PACKET_BYTES,
            _ => LORA_MAX_PACKET_BYTES,
        }
    }

    /// Approximate one-way end-to-end frame latency (UART serialisation + air time + MCU turnaround)
    /// for a `frame_bytes` COBS packet at `uart_bps` (8N1 = 10 bits/byte).
    pub const fn estimate_one_way_ms(self, frame_bytes: usize, uart_bps: u32) -> u32 {
        let bits = (frame_bytes as u64) * 10;
        let uart_ms = if uart_bps > 0 {
            // TX UART into sender module + RX UART out of receiver module.
            (bits * 2_000).div_ceil(uart_bps as u64)
        } else {
            0
        };
        let payload_bits = ((frame_bytes as u64) + 8) * 8;
        let air_bps = self.air_bps() as u64;
        let air_ms = (payload_bits * 1_000).div_ceil(air_bps);
        // 4 ms accounts for DX-LR42 MCU packet framing + 2-3 ms AUX pre-wake turnaround.
        (uart_ms + air_ms + 4) as u32
    }
}

/// Convert a `DX-LR42-24T12D` hex channel index (`0x00..=0x32`) to carrier frequency in MHz.
pub const fn channel_freq_mhz(channel: u8) -> Option<u16> {
    if channel > MAX_CHANNEL {
        None
    } else {
        Some(BASE_FREQ_MHZ + (channel as u16) * 2)
    }
}

/// Convert an even `2400..=2500 MHz` carrier frequency into a `DX-LR42-24T12D` channel index.
pub const fn channel_from_freq_mhz(freq_mhz: u16) -> Option<u8> {
    if freq_mhz < BASE_FREQ_MHZ || freq_mhz > 2_500 || (freq_mhz - BASE_FREQ_MHZ) % 2 != 0 {
        None
    } else {
        Some(((freq_mhz - BASE_FREQ_MHZ) / 2) as u8)
    }
}

/// Convert the trailing byte appended when `AT+DRSSI1` is enabled into RSSI in dBm.
///
/// Datasheet section 5.3.11: `RSSI_dBm = -(0xFF - raw_byte)` (e.g. `0xAB -> -84 dBm`).
pub const fn decode_drssi_dbm(raw_byte: u8) -> i16 {
    -(0xFF_i16 - (raw_byte as i16))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_channel_and_bounds_match_datasheet() {
        assert_eq!(channel_freq_mhz(0x00), Some(2_400));
        assert_eq!(channel_freq_mhz(DEFAULT_CHANNEL), Some(2_424));
        assert_eq!(channel_freq_mhz(MAX_CHANNEL), Some(2_500));
        assert_eq!(channel_freq_mhz(0x33), None);

        assert_eq!(channel_from_freq_mhz(2_424), Some(0x0C));
        assert_eq!(channel_from_freq_mhz(2_400), Some(0x00));
        assert_eq!(channel_from_freq_mhz(2_500), Some(0x32));
        assert_eq!(channel_from_freq_mhz(2_425), None);
        assert_eq!(channel_from_freq_mhz(2_398), None);
    }

    #[test]
    fn drssi_formula_matches_manual_worked_example() {
        // Worked example in DX-LR42 AT manual section 5.3.11: 0xAB -> -(255 - 171) = -84 dBm.
        assert_eq!(decode_drssi_dbm(0xAB), -84);
        assert_eq!(decode_drssi_dbm(0xFF), 0);
        assert_eq!(decode_drssi_dbm(0x00), -255);
    }

    #[test]
    fn baud_and_level_tables_roundtrip() {
        assert_eq!(DxLr42Baud::default().bps(), crate::RADIO_BAUD);
        for code in 0..=6 {
            let b = DxLr42Baud::from_code(code).unwrap();
            assert_eq!(b.code(), code);
            assert_eq!(DxLr42Baud::from_bps(b.bps()), Some(b));
        }
        assert_eq!(DxLr42Baud::from_code(7), None);
        assert_eq!(DxLr42Baud::from_bps(14_400), None);

        assert_eq!(DxLr42Level::default(), DxLr42Level::Level3);
        assert_eq!(DxLr42Level::Level3.air_bps(), 10_000);
        assert_eq!(DxLr42Level::Level1.air_bps(), 1_000);
        assert_eq!(DxLr42Level::Level8.max_packet_bytes(), 127);
        assert_eq!(DxLr42Level::Level3.max_packet_bytes(), 230);
        for code in 0..=8 {
            assert_eq!(DxLr42Level::from_code(code).unwrap().code(), code);
        }
        assert_eq!(DxLr42Level::from_code(9), None);

        // Typical 25-byte STATE frame at default 9600 Bd + LEVEL 3 (10 kbps) is ~82 ms end-to-end.
        let one_way = DxLr42Level::Level3.estimate_one_way_ms(25, 9_600);
        assert!((60..=110).contains(&one_way), "got {one_way} ms");
    }
}
