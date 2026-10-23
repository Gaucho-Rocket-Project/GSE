//! 2.4 GHz Semtech SX1280 / SX1281 radio module definitions, baud rate presets,
//! and over-the-air link timing helpers.
//!
//! Two hardware transceiver options are supported on the NUCLEO-H753ZI boards:
//!
//! 1. **DX-SMART `DX-LR42-24T12D`** (`RadioModule::DxLr42`, factory default):
//!    * Interface: `3.3 V` CMOS UART (`USART6`: `PG14` TX, `PG9` RX, `PG10` AUX,
//!      `PG12` M0, `PG13` M1), selectable `2400..=115200 Bd 8N1` (`AT+BAUD0..6`,
//!      default `9600 Bd`).
//!    * RF profile: `2424 MHz` (`AT+CHANNEL0C`), `SF8 / BW 406 kHz / CR 4/5`
//!      (`AT+LEVEL3`, `10 kbps` nominal), `+13 dBm` (`20 mW`) at `VCC = 5.0 V`.
//!
//! 2. **CDEBYTE `E28-2G4M27S`** (`RadioModule::E28_2G4M27S`, high-power `500 mW`):
//!    * Interface: `3.3 V` 4-wire `SPI1` (`PA5` SCK, `PA6` MISO, `PA7` MOSI,
//!      `PD14` NSS at `4 MHz` Mode 0) + `PG10` `BUSY`, `PG11` `NRESET`,
//!      `PF13` `DIO1`, `PG12` `RX_EN` (LNA), `PG13` `TX_EN` (`500 mW` PA).
//!    * RF profile: `2424 MHz` (`52 MHz` XOSC), LoRa `SF8 / BW 406.25 kHz / CR 4/5`,
//!      SX1280/SX1281 internal `0 dBm` (`power = 18`) driving onboard PA to full
//!      `+27 dBm` (`500 mW`), `-132 dBm` LNA sensitivity.
//!    * **Supply voltage warning**: strictly `VCC = 2.5–3.6 V` (`3.3 V` typ,
//!      `>= 580 mA` peak TX); applying `5 V` permanently damages `E28-2G4M27S`.

use crate::frame::{CRC_LEN, HEADER_LEN, MAX_ENCODED_LEN};

/// Default installed 2.4 GHz SX1281 UART transceiver part number.
pub const MODULE_PART: &str = "DX-LR42-24T12D";

/// High-power 2.4 GHz SX1280/SX1281 + PA/LNA (`500 mW` / `+27 dBm`) SPI transceiver part number.
pub const E28_MODULE_PART: &str = "E28-2G4M27S";

/// Minimum possible wire length of a valid GSE frame (`0x00` + COBS(header + crc) + `0x00`).
pub const MIN_ENCODED_FRAME_LEN: usize = HEADER_LEN + CRC_LEN + 3;

/// Maximum bytes in a single LoRa packet on the `DX-LR42-24T12D` (`AT+PACKET3`, default).
pub const LORA_MAX_PACKET_BYTES: usize = 230;

/// Fixed sub-packet size in `LEVEL 8` FLRC mode (`1.04 Mbps`).
pub const FLRC_MAX_PACKET_BYTES: usize = 127;

/// Minimum payload required by `DX-LR42-24T12D` in `LEVEL 8` FLRC mode (`>= 6 B`).
pub const FLRC_MIN_PACKET_BYTES: usize = 6;

// Compile-time proof that every valid GSE frame fits in a single air packet across
// all 9 `DX-LR42-24T12D` modulation levels (LoRa `0..=7` and FLRC `8`) as well as
// the 128-byte TX/RX split FIFO on `E28-2G4M27S` (`SX1280`/`SX1281`).
const _: () = {
    assert!(MIN_ENCODED_FRAME_LEN >= FLRC_MIN_PACKET_BYTES);
    assert!(MAX_ENCODED_LEN <= FLRC_MAX_PACKET_BYTES);
    assert!(MAX_ENCODED_LEN <= e28::FIFO_HALF_BYTES);
    assert!(FLRC_MAX_PACKET_BYTES <= LORA_MAX_PACKET_BYTES);
};

/// Selected radio transceiver hardware on the NUCLEO-H753ZI boards.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RadioModule {
    /// DX-SMART `DX-LR42-24T12D` (`SX1281`, `USART6` transparent UART, `+13 dBm` / `20 mW`).
    #[default]
    DxLr42 = 0,
    /// CDEBYTE `E28-2G4M27S` (`SX1280`/`SX1281` + PA/LNA, `SPI1` `4 MHz`, `+27 dBm` / `500 mW`).
    E28_2G4M27S = 1,
}

impl RadioModule {
    pub const ALL: [RadioModule; 2] = [RadioModule::DxLr42, RadioModule::E28_2G4M27S];

    pub const fn code(self) -> u8 {
        self as u8
    }

    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(RadioModule::DxLr42),
            1 => Some(RadioModule::E28_2G4M27S),
            _ => None,
        }
    }

    /// Stable machine identifier used in JSON configuration and HTTP API payloads.
    pub const fn as_str(self) -> &'static str {
        match self {
            RadioModule::DxLr42 => "dx-lr42-24t12d",
            RadioModule::E28_2G4M27S => "e28-2g4m27s",
        }
    }

    /// Alias for [`RadioModule::as_str`].
    pub const fn id(self) -> &'static str {
        self.as_str()
    }

    /// Parse a case-insensitive identifier or part number alias.
    pub fn parse(text: &str) -> Option<Self> {
        let s = text.trim();
        if s.eq_ignore_ascii_case("dx-lr42-24t12d")
            || s.eq_ignore_ascii_case("dx-lr42")
            || s.eq_ignore_ascii_case("dxlr42")
            || s == "0"
        {
            Some(RadioModule::DxLr42)
        } else if s.eq_ignore_ascii_case("e28-2g4m27s")
            || s.eq_ignore_ascii_case("e28_2g4m27s")
            || s.eq_ignore_ascii_case("e28")
            || s == "1"
        {
            Some(RadioModule::E28_2G4M27S)
        } else {
            None
        }
    }

    /// Part number shown in compact telemetry badges.
    pub const fn part_number(self) -> &'static str {
        match self {
            RadioModule::DxLr42 => MODULE_PART,
            RadioModule::E28_2G4M27S => E28_MODULE_PART,
        }
    }

    /// Full operator-facing label for dropdown menus.
    pub const fn label(self) -> &'static str {
        match self {
            RadioModule::DxLr42 => "DX-LR42-24T12D (UART · SX1281 · 13 dBm)",
            RadioModule::E28_2G4M27S => "E28-2G4M27S (SPI1 · SX1281 · 27 dBm / 500 mW)",
        }
    }

    /// Host MCU bus peripheral used to drive this module.
    pub const fn bus_kind(self) -> &'static str {
        match self {
            RadioModule::DxLr42 => "USART6",
            RadioModule::E28_2G4M27S => "SPI1",
        }
    }

    /// True when the active radio uses the `USART6` UART baud rate setting.
    pub const fn uses_uart(self) -> bool {
        matches!(self, RadioModule::DxLr42)
    }

    /// Nominal maximum conducted RF output power in dBm.
    pub const fn max_power_dbm(self) -> u8 {
        match self {
            RadioModule::DxLr42 => MAX_POWER_DBM,
            RadioModule::E28_2G4M27S => e28::MAX_POWER_DBM,
        }
    }
}

/// Supported USB CDC (`USART3` ST-LINK Virtual COM Port) baud rate presets (`8N1`).
///
/// Every preset is natively supported across Linux (`termios`), macOS (`termios` /
/// `IOSSIOSPEED`), Windows (`SetCommState`), and STM32H753ZI `USART3` (`64 MHz` HSI).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum UsbBaud {
    Baud9600 = 0,
    Baud19200 = 1,
    Baud38400 = 2,
    Baud57600 = 3,
    #[default]
    Baud115200 = 4,
    Baud230400 = 5,
    Baud460800 = 6,
}

impl UsbBaud {
    pub const ALL: [UsbBaud; 7] = [
        UsbBaud::Baud9600,
        UsbBaud::Baud19200,
        UsbBaud::Baud38400,
        UsbBaud::Baud57600,
        UsbBaud::Baud115200,
        UsbBaud::Baud230400,
        UsbBaud::Baud460800,
    ];

    pub const fn code(self) -> u8 {
        self as u8
    }

    pub const fn bps(self) -> u32 {
        match self {
            UsbBaud::Baud9600 => 9_600,
            UsbBaud::Baud19200 => 19_200,
            UsbBaud::Baud38400 => 38_400,
            UsbBaud::Baud57600 => 57_600,
            UsbBaud::Baud115200 => 115_200,
            UsbBaud::Baud230400 => 230_400,
            UsbBaud::Baud460800 => 460_800,
        }
    }

    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(UsbBaud::Baud9600),
            1 => Some(UsbBaud::Baud19200),
            2 => Some(UsbBaud::Baud38400),
            3 => Some(UsbBaud::Baud57600),
            4 => Some(UsbBaud::Baud115200),
            5 => Some(UsbBaud::Baud230400),
            6 => Some(UsbBaud::Baud460800),
            _ => None,
        }
    }

    pub const fn from_bps(bps: u32) -> Option<Self> {
        match bps {
            9_600 => Some(UsbBaud::Baud9600),
            19_200 => Some(UsbBaud::Baud19200),
            38_400 => Some(UsbBaud::Baud38400),
            57_600 => Some(UsbBaud::Baud57600),
            115_200 => Some(UsbBaud::Baud115200),
            230_400 => Some(UsbBaud::Baud230400),
            460_800 => Some(UsbBaud::Baud460800),
            _ => None,
        }
    }
}

/// Factory default RF channel (`0x0C` = `2424 MHz`).
pub const DEFAULT_CHANNEL: u8 = 0x0C;

/// Highest supported RF channel (`0x32` = `50` decimal = `2500 MHz`).
pub const MAX_CHANNEL: u8 = 0x32;

/// Base frequency for channel `0x00`, in MHz (`2 MHz` step per channel).
pub const BASE_FREQ_MHZ: u16 = 2_400;

/// Factory default and maximum RF output power on `DX-LR42-24T12D`, in dBm (`+13 dBm` at `VCC >= 5.0 V`).
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

/// Supported `USART6` radio UART speed presets (`AT+BAUD0` .. `AT+BAUD6`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
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
    pub const ALL: [DxLr42Baud; 7] = [
        DxLr42Baud::Baud2400,
        DxLr42Baud::Baud4800,
        DxLr42Baud::Baud9600,
        DxLr42Baud::Baud19200,
        DxLr42Baud::Baud38400,
        DxLr42Baud::Baud57600,
        DxLr42Baud::Baud115200,
    ];

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

    /// Exact `DX-LR42-24T12D` AT command string that persists this UART baud in EEPROM.
    pub const fn at_command(self) -> &'static [u8] {
        match self {
            DxLr42Baud::Baud2400 => b"AT+BAUD0\r\n",
            DxLr42Baud::Baud4800 => b"AT+BAUD1\r\n",
            DxLr42Baud::Baud9600 => b"AT+BAUD2\r\n",
            DxLr42Baud::Baud19200 => b"AT+BAUD3\r\n",
            DxLr42Baud::Baud38400 => b"AT+BAUD4\r\n",
            DxLr42Baud::Baud57600 => b"AT+BAUD5\r\n",
            DxLr42Baud::Baud115200 => b"AT+BAUD6\r\n",
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

/// CDEBYTE `E28-2G4M27S` (`Semtech SX1280` / `SX1281` + PA/LNA `+27 dBm` / `500 mW`)
/// register addresses, SPI opcodes, modulation constants, and timing helpers.
pub mod e28 {
    use crate::frame::MAX_ENCODED_LEN;

    /// Crystal oscillator frequency on `E28-2G4M27S` (`52.000 MHz` `±10 ppm`).
    pub const XTAL_HZ: u64 = 52_000_000;

    /// `SX1280`/`SX1281` fractional PLL step (`2^18`).
    pub const PLL_DIVIDER: u64 = 1 << 18;

    /// Default carrier frequency (`2424.000 MHz`, matching channel `0x0C`).
    pub const DEFAULT_RF_HZ: u32 = 2_424_000_000;

    /// Precomputed 24-bit PLL word for `2424.000 MHz` (`[0xBA, 0x76, 0x27]`).
    pub const DEFAULT_RF_PLL: [u8; 3] = freq_hz_to_pll_steps(DEFAULT_RF_HZ);

    /// `SPI1` clock frequency driven by STM32H753ZI (`64 MHz / 16 = 4.000 MHz`; max `10 MHz`).
    pub const SPI_CLOCK_HZ: u32 = 4_000_000;

    /// Maximum RF output power with onboard PA enabled (`+27 dBm` / `500 mW` at `3.3 V`).
    pub const MAX_POWER_DBM: u8 = 27;

    /// SX1280/SX1281 internal `SetTxParams` power byte (`P_ic = -18 + 18 = 0 dBm`),
    /// recommended by CDEBYTE `E28-2G4M27S` manual §3 so the external `500 mW` PA
    /// delivers rated `+27 dBm` output cleanly without saturating supply current.
    pub const IC_TX_POWER_REG: u8 = 18;

    /// Split `256 B` data buffer evenly between TX (`0x00..0x7F`) and RX (`0x80..0xFF`).
    pub const TX_BASE_ADDR: u8 = 0x00;
    pub const RX_BASE_ADDR: u8 = 0x80;
    pub const FIFO_HALF_BYTES: usize = 128;

    /// LoRa preamble length in symbols (`12` symbols per Semtech `SX1280` DS §14.4).
    pub const LORA_PREAMBLE_SYMBOLS: u8 = 12;

    /// Max LoRa packet length programmed into `REG_LR_PAYLOADLENGTH` (`0x40` = `64` bytes).
    pub const MAX_PACKET_LEN: u8 = MAX_ENCODED_LEN as u8;

    // --- Semtech SX1280 / SX1281 SPI opcodes (Mode 0, MSB-first) ---
    pub const CMD_GET_STATUS: u8 = 0xC0;
    pub const CMD_WRITE_REGISTER: u8 = 0x18;
    pub const CMD_READ_REGISTER: u8 = 0x19;
    pub const CMD_WRITE_BUFFER: u8 = 0x1A;
    pub const CMD_READ_BUFFER: u8 = 0x1B;
    pub const CMD_SET_SLEEP: u8 = 0x84;
    pub const CMD_SET_STANDBY: u8 = 0x80;
    pub const CMD_SET_FS: u8 = 0xC1;
    pub const CMD_SET_TX: u8 = 0x83;
    pub const CMD_SET_RX: u8 = 0x82;
    pub const CMD_SET_RF_FREQUENCY: u8 = 0x86;
    pub const CMD_SET_CAD_PARAMS: u8 = 0x88;
    pub const CMD_CALIBRATE: u8 = 0x89;
    pub const CMD_SET_PACKET_TYPE: u8 = 0x8A;
    pub const CMD_SET_MODULATION_PARAMS: u8 = 0x8B;
    pub const CMD_SET_PACKET_PARAMS: u8 = 0x8C;
    pub const CMD_SET_DIO_IRQ_PARAMS: u8 = 0x8D;
    pub const CMD_SET_TX_PARAMS: u8 = 0x8E;
    pub const CMD_SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;
    pub const CMD_SET_REGULATOR_MODE: u8 = 0x96;
    pub const CMD_CLR_IRQ_STATUS: u8 = 0x97;
    pub const CMD_GET_IRQ_STATUS: u8 = 0x15;
    pub const CMD_GET_RX_BUFFER_STATUS: u8 = 0x17;
    pub const CMD_GET_PACKET_STATUS: u8 = 0x1D;
    pub const CMD_GET_RSSI_INST: u8 = 0x1F;

    // --- Standby / regulator / packet modes ---
    pub const STDBY_RC: u8 = 0x00;
    pub const STDBY_XOSC: u8 = 0x01;
    pub const REGULATOR_LDO: u8 = 0x00;
    pub const REGULATOR_DCDC: u8 = 0x01;
    pub const CALIBRATE_ALL: u8 = 0x7F;
    pub const PACKET_TYPE_GFSK: u8 = 0x00;
    pub const PACKET_TYPE_LORA: u8 = 0x01;
    pub const PACKET_TYPE_FLRC: u8 = 0x03;

    // --- LoRa modulation & packet parameters (`SX1280` DS Table 14-47..14-52) ---
    pub const LORA_SF5: u8 = 0x50;
    pub const LORA_SF6: u8 = 0x60;
    pub const LORA_SF7: u8 = 0x70;
    pub const LORA_SF8: u8 = 0x80;
    pub const LORA_SF9: u8 = 0x90;
    pub const LORA_SF10: u8 = 0xA0;
    pub const LORA_SF11: u8 = 0xB0;
    pub const LORA_SF12: u8 = 0xC0;

    pub const LORA_BW_0200: u8 = 0x34; // 203.125 kHz
    pub const LORA_BW_0400: u8 = 0x26; // 406.250 kHz
    pub const LORA_BW_0800: u8 = 0x18; // 812.500 kHz
    pub const LORA_BW_1600: u8 = 0x0A; // 1625.000 kHz

    pub const LORA_CR_4_5: u8 = 0x01;
    pub const LORA_CR_4_6: u8 = 0x02;
    pub const LORA_CR_4_7: u8 = 0x03;
    pub const LORA_CR_4_8: u8 = 0x04;

    pub const LORA_PACKET_EXPLICIT: u8 = 0x00;
    pub const LORA_PACKET_IMPLICIT: u8 = 0x80;
    pub const LORA_CRC_ON: u8 = 0x20;
    pub const LORA_CRC_OFF: u8 = 0x00;
    pub const LORA_IQ_NORMAL: u8 = 0x40;
    pub const LORA_IQ_INVERTED: u8 = 0x00;

    pub const RADIO_RAMP_20_US: u8 = 0xE0;
    pub const PERIOD_BASE_1_MS: u8 = 0x02;
    pub const RX_CONTINUOUS_STEPS: u16 = 0xFFFF;
    pub const TX_TIMEOUT_MS: u16 = 250;

    // --- SX1280 / SX1281 internal registers ---
    pub const REG_LNA_REGIME: u16 = 0x0891;
    pub const REG_LR_PAYLOAD_LENGTH: u16 = 0x0901;
    pub const REG_LR_SF_ADDITIONAL: u16 = 0x0925;
    pub const REG_LR_FREQ_ERR_COMP: u16 = 0x093C;
    pub const REG_LR_SYNC_WORD_MSB: u16 = 0x0944;

    // --- SX1280 / SX1281 IRQ status bitmasks ---
    pub const IRQ_TX_DONE: u16 = 0x0001;
    pub const IRQ_RX_DONE: u16 = 0x0002;
    pub const IRQ_HEADER_ERROR: u16 = 0x0020;
    pub const IRQ_CRC_ERROR: u16 = 0x0040;
    pub const IRQ_RX_TX_TIMEOUT: u16 = 0x4000;
    pub const IRQ_DEFAULT_MASK: u16 =
        IRQ_TX_DONE | IRQ_RX_DONE | IRQ_HEADER_ERROR | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;

    /// Convert carrier frequency in Hz to 24-bit big-endian SX1280/SX1281 `SetRfFrequency` bytes.
    pub const fn freq_hz_to_pll_steps(freq_hz: u32) -> [u8; 3] {
        let steps = ((freq_hz as u64) * PLL_DIVIDER) / XTAL_HZ;
        [
            ((steps >> 16) & 0xFF) as u8,
            ((steps >> 8) & 0xFF) as u8,
            (steps & 0xFF) as u8,
        ]
    }

    /// Convert 24-bit big-endian `SetRfFrequency` bytes back to carrier frequency in Hz.
    pub const fn pll_steps_to_freq_hz(pll: [u8; 3]) -> u32 {
        let steps = ((pll[0] as u64) << 16) | ((pll[1] as u64) << 8) | (pll[2] as u64);
        ((steps * XTAL_HZ) / PLL_DIVIDER) as u32
    }

    /// Required `0x0925` register trim value after `SetModulationParams` (`SX1280` DS §14.4.1).
    pub const fn sf_additional_reg(sf_param: u8) -> u8 {
        match sf_param {
            LORA_SF5 | LORA_SF6 => 0x1E,
            LORA_SF7 | LORA_SF8 => 0x37,
            _ => 0x32,
        }
    }

    /// Estimate one-way over-the-air + `4 MHz` SPI packet latency (`ms`) for `E28-2G4M27S`
    /// in default `SF8 / BW 406.25 kHz / CR 4/5` mode (`~15.6 kbps` effective LoRa rate).
    pub const fn estimate_one_way_ms(frame_bytes: usize) -> u32 {
        let payload_bits = ((frame_bytes as u64) + 10) * 8;
        let air_ms = (payload_bits * 1_000).div_ceil(15_625);
        // ~1 ms total 4 MHz SPI burst + SX1281 TX/RX state switch.
        (air_ms + 2) as u32
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
            assert!(b.at_command().starts_with(b"AT+BAUD"));
            assert!(b.at_command().ends_with(b"\r\n"));
        }
        assert_eq!(DxLr42Baud::from_code(7), None);
        assert_eq!(DxLr42Baud::from_bps(14_400), None);

        assert_eq!(UsbBaud::default().bps(), crate::USB_BAUD);
        for code in 0..=6 {
            let u = UsbBaud::from_code(code).unwrap();
            assert_eq!(u.code(), code);
            assert_eq!(UsbBaud::from_bps(u.bps()), Some(u));
        }
        assert_eq!(UsbBaud::from_code(7), None);
        assert_eq!(UsbBaud::from_bps(12_345), None);

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

    #[test]
    fn radio_module_and_e28_pll_math_roundtrip() {
        for m in RadioModule::ALL {
            assert_eq!(RadioModule::from_code(m.code()), Some(m));
            assert_eq!(RadioModule::parse(m.as_str()), Some(m));
            assert_eq!(RadioModule::parse(m.part_number()), Some(m));
        }
        assert_eq!(RadioModule::from_code(2), None);
        assert_eq!(RadioModule::parse("bogus"), None);
        assert!(RadioModule::DxLr42.uses_uart());
        assert!(!RadioModule::E28_2G4M27S.uses_uart());
        assert_eq!(RadioModule::DxLr42.max_power_dbm(), 13);
        assert_eq!(RadioModule::E28_2G4M27S.max_power_dbm(), 27);

        // 2424.000 MHz -> [0xBA, 0x76, 0x27], round-trip error < 200 Hz (< 0.1 ppm).
        assert_eq!(e28::DEFAULT_RF_PLL, [0xBA, 0x76, 0x27]);
        let back_hz = e28::pll_steps_to_freq_hz(e28::DEFAULT_RF_PLL);
        assert!(back_hz.abs_diff(e28::DEFAULT_RF_HZ) < 200);
        // SPI-driven E28 skips slow 9600 Bd UART framing (~20 ms vs ~82 ms).
        let spi_ms = e28::estimate_one_way_ms(25);
        assert!((15..=30).contains(&spi_ms), "got {spi_ms} ms");
    }
}
