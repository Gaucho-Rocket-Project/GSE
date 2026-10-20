//! CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, no reflection, no xor-out).
//!
//! Bitwise implementation: the firmware sends at most a few hundred bytes per
//! second, so table-free code is preferable to a 512 byte lookup table.

pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_vector() {
        // Standard CRC-16/CCITT-FALSE check value.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn empty_is_init() {
        assert_eq!(crc16(&[]), 0xFFFF);
    }

    #[test]
    fn single_bit_flip_is_detected() {
        let a = crc16(b"GSE command fill on");
        let b = crc16(b"GSE command fill oN");
        assert_ne!(a, b);
    }
}
