//! Consistent Overhead Byte Stuffing (COBS).
//!
//! Frames are delimited by `0x00`. COBS guarantees that no `0x00` appears
//! inside the encoded block, so a receiver can always resynchronise after a
//! dropped byte or a corrupted frame.

/// Worst-case encoded length for `n` input bytes (not including delimiters).
pub const fn max_encoded_len(n: usize) -> usize {
    n + n / 254 + 1
}

/// Worst-case decoded length for `n` encoded bytes.
pub const fn max_decoded_len(n: usize) -> usize {
    n
}

/// Encode `data` into `out`. Returns the number of bytes written.
///
/// `out` must be at least `max_encoded_len(data.len())` bytes long.
pub fn encode(data: &[u8], out: &mut [u8]) -> Option<usize> {
    if out.len() < max_encoded_len(data.len()) {
        return None;
    }
    // First byte of a block is always a code byte (a zero-length input still
    // produces the single byte 0x01).
    let mut write = 1usize;
    let mut code_index = 0usize;
    let mut code: u8 = 1;

    for &byte in data {
        if byte == 0 {
            out[code_index] = code;
            code_index = write;
            write += 1;
            code = 1;
        } else {
            out[write] = byte;
            write += 1;
            code += 1;
            if code == 0xFF {
                out[code_index] = code;
                code_index = write;
                write += 1;
                code = 1;
            }
        }
    }
    out[code_index] = code;
    Some(write)
}

/// Decode one COBS block into `out`. Returns the decoded length.
pub fn decode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut read = 0usize;
    let mut write = 0usize;
    while read < input.len() {
        let code = input[read] as usize;
        if code == 0 {
            // A literal zero is illegal inside a COBS block.
            return None;
        }
        read += 1;
        let end = read + code - 1;
        if end > input.len() {
            return None;
        }
        if write + (code - 1) > out.len() {
            return None;
        }
        out[write..write + code - 1].copy_from_slice(&input[read..end]);
        write += code - 1;
        read = end;
        if code != 0xFF && read < input.len() {
            if write >= out.len() {
                return None;
            }
            out[write] = 0;
            write += 1;
        }
    }
    Some(write)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(data: &[u8]) -> Vec<u8> {
        let mut enc = [0u8; 1024];
        let n = encode(data, &mut enc).expect("encode");
        assert!(!enc[..n].contains(&0), "encoded block contains a zero");
        let mut dec = [0u8; 1024];
        let m = decode(&enc[..n], &mut dec).expect("decode");
        assert_eq!(&dec[..m], data);
        dec[..m].to_vec()
    }

    #[test]
    fn empty() {
        assert_eq!(roundtrip(&[]), Vec::<u8>::new());
    }

    #[test]
    fn simple() {
        assert_eq!(roundtrip(b"hello world"), b"hello world");
    }

    #[test]
    fn zeros() {
        assert_eq!(roundtrip(&[0, 0, 1, 0, 2, 0, 0]), vec![0, 0, 1, 0, 2, 0, 0]);
        assert_eq!(roundtrip(&[0; 32]), vec![0; 32]);
    }

    #[test]
    fn long_run_without_zeros() {
        let data = vec![0xAAu8; 400];
        assert_eq!(roundtrip(&data), data);
    }

    #[test]
    fn mixed_long() {
        let mut data = Vec::new();
        for i in 0..600u32 {
            data.push(if i % 7 == 0 { 0 } else { (i % 251) as u8 });
        }
        assert_eq!(roundtrip(&data), data);
    }

    #[test]
    fn buffer_too_small() {
        let mut out = [0u8; 3];
        assert!(encode(&[1, 2, 3], &mut out).is_none());
        let mut small = [0u8; 2];
        assert!(decode(&[2, 1, 2], &mut small).is_none());
    }

    #[test]
    fn rejects_zero_inside_block() {
        let mut out = [0u8; 8];
        assert!(decode(&[2, 0, 2], &mut out).is_none());
    }

    #[test]
    fn rejects_truncated_block() {
        let mut out = [0u8; 8];
        assert!(decode(&[5, 1], &mut out).is_none());
    }
}
