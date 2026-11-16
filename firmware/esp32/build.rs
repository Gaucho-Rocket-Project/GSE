//! Generates the ESP-NOW link configuration from build-time environment
//! variables, so peer addresses and keys are never hard-coded in the source.

use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

/// Locally administered unicast station addresses each board assigns itself.
const DEFAULT_GROUND_MAC: &str = "02:47:53:45:00:01";
const DEFAULT_VEHICLE_MAC: &str = "02:47:53:45:00:02";
const DEFAULT_CHANNEL: &str = "1";

fn main() {
    embuild::espidf::sysenv::output();

    let ground_mac = mac("GSE_GROUND_MAC", DEFAULT_GROUND_MAC);
    let vehicle_mac = mac("GSE_VEHICLE_MAC", DEFAULT_VEHICLE_MAC);
    if ground_mac == vehicle_mac {
        panic!("GSE_GROUND_MAC and GSE_VEHICLE_MAC must differ");
    }
    let pmk = key("GSE_ESPNOW_PMK");
    let lmk = key("GSE_ESPNOW_LMK");
    let channel: u8 = var("GSE_ESPNOW_CHANNEL")
        .unwrap_or_else(|| DEFAULT_CHANNEL.into())
        .parse()
        .ok()
        .filter(|c| (1..=13).contains(c))
        .unwrap_or_else(|| panic!("GSE_ESPNOW_CHANNEL must be a Wi-Fi channel from 1 to 13"));

    let mut out = String::new();
    writeln!(out, "pub const GROUND_MAC: [u8; 6] = {ground_mac:?};").unwrap();
    writeln!(out, "pub const VEHICLE_MAC: [u8; 6] = {vehicle_mac:?};").unwrap();
    writeln!(out, "pub const ESPNOW_PMK: [u8; 16] = {pmk:?};").unwrap();
    writeln!(out, "pub const ESPNOW_LMK: [u8; 16] = {lmk:?};").unwrap();
    writeln!(out, "pub const ESPNOW_CHANNEL: u8 = {channel};").unwrap();
    let path = Path::new(&env::var("OUT_DIR").unwrap()).join("link_config.rs");
    fs::write(path, out).unwrap();
}

fn var(name: &str) -> Option<String> {
    println!("cargo:rerun-if-env-changed={name}");
    env::var(name).ok().filter(|v| !v.trim().is_empty())
}

fn mac(name: &str, default: &str) -> [u8; 6] {
    let text = var(name).unwrap_or_else(|| default.into());
    let bytes: Vec<u8> = text
        .trim()
        .split([':', '-'])
        .map(|b| u8::from_str_radix(b, 16))
        .collect::<Result<_, _>>()
        .unwrap_or_else(|_| panic!("{name} must look like 02:47:53:45:00:01, got {text:?}"));
    let mac: [u8; 6] = bytes
        .try_into()
        .unwrap_or_else(|_| panic!("{name} must have six bytes, got {text:?}"));
    if mac[0] & 0x01 != 0 {
        panic!("{name} must be a unicast address (first byte even), got {text:?}");
    }
    mac
}

fn key(name: &str) -> [u8; 16] {
    let Some(text) = var(name) else {
        panic!(
            "{name} is not set. ESP-NOW encryption is required; generate both keys once with \
             `openssl rand -hex 16`, then build both boards with the same \
             GSE_ESPNOW_PMK and GSE_ESPNOW_LMK (see firmware/esp32/README.md)."
        );
    };
    let text = text.trim();
    if text.len() != 32 {
        panic!("{name} must be 32 hex characters (16 bytes)");
    }
    let mut key = [0u8; 16];
    for (i, byte) in key.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&text[i * 2..i * 2 + 2], 16)
            .unwrap_or_else(|_| panic!("{name} must be 32 hex characters (16 bytes)"));
    }
    key
}
