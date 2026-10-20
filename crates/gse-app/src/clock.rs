//! Wall-clock helpers (no `chrono` dependency).
//!
//! The UI receives Unix epoch milliseconds and formats local time in the
//! browser; log files are written in ISO-8601 UTC so sessions from different
//! machines can be correlated.

use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub fn unix_ms() -> u64 {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => d.as_millis() as u64,
        Err(_) => 0,
    }
}

pub fn elapsed_ms(start: Instant) -> u64 {
    Instant::now().saturating_duration_since(start).as_millis() as u64
}

/// Convert Unix epoch milliseconds to `YYYY-MM-DDTHH:MM:SS.mmmZ`.
pub fn iso8601_utc(epoch_ms: u64) -> String {
    let secs = (epoch_ms / 1000) as i64;
    let millis = (epoch_ms % 1000) as u32;
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    let hour = rem / 3600;
    let minute = (rem % 3600) / 60;
    let second = rem % 60;
    format!("{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}.{millis:03}Z")
}

/// Howard Hinnant's `civil_from_days` algorithm.
fn civil_from_days(days_since_epoch: i64) -> (i64, u32, u32) {
    let z = days_since_epoch + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// Human-friendly duration, e.g. `1h 02m`, `3m 07s`, `42.3s`.
pub fn short_duration(ms: u64) -> String {
    let total_s = ms / 1000;
    if total_s >= 3600 {
        format!("{}h {:02}m", total_s / 3600, (total_s % 3600) / 60)
    } else if total_s >= 60 {
        format!("{}m {:02}s", total_s / 60, total_s % 60)
    } else if total_s >= 10 {
        format!("{total_s}s")
    } else {
        format!("{:.1}s", ms as f64 / 1000.0)
    }
}

pub fn duration_from_ms(ms: u64) -> Duration {
    Duration::from_millis(ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epoch_is_1970() {
        assert_eq!(iso8601_utc(0), "1970-01-01T00:00:00.000Z");
    }

    #[test]
    fn known_timestamps() {
        assert_eq!(iso8601_utc(946_684_800_000), "2000-01-01T00:00:00.000Z");
        assert_eq!(iso8601_utc(1_700_000_000_123), "2023-11-14T22:13:20.123Z");
        assert_eq!(iso8601_utc(1_709_164_800_000), "2024-02-29T00:00:00.000Z");
        assert_eq!(iso8601_utc(1_735_689_599_999), "2024-12-31T23:59:59.999Z");
    }

    #[test]
    fn durations() {
        assert_eq!(short_duration(4_230), "4.2s");
        assert_eq!(short_duration(42_300), "42s");
        assert_eq!(short_duration(15_000), "15s");
        assert_eq!(short_duration(187_000), "3m 07s");
        assert_eq!(short_duration(3_723_000), "1h 02m");
    }

    #[test]
    fn clock_moves() {
        let now = unix_ms();
        assert!(now > 1_600_000_000_000);
    }
}
