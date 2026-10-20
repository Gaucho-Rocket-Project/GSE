//! Event log entries and the per-session log file.

use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};

use crate::clock;
use crate::json;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Info,
    Ok,
    Warn,
    Error,
}

impl Level {
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Info => "info",
            Level::Ok => "ok",
            Level::Warn => "warn",
            Level::Error => "error",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Level::Info => "INFO",
            Level::Ok => " OK ",
            Level::Warn => "WARN",
            Level::Error => "ERR ",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    App,
    Operator,
    Ground,
    Vehicle,
}

impl Source {
    pub fn as_str(self) -> &'static str {
        match self {
            Source::App => "app",
            Source::Operator => "operator",
            Source::Ground => "ground",
            Source::Vehicle => "vehicle",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Source::App => "APP",
            Source::Operator => "OPS",
            Source::Ground => "GND",
            Source::Vehicle => "VEH",
        }
    }
}

#[derive(Clone, Debug)]
pub struct LogEntry {
    pub id: u64,
    pub epoch_ms: u64,
    pub level: Level,
    pub source: Source,
    pub message: String,
    pub code: Option<&'static str>,
}

impl LogEntry {
    pub fn to_json(&self) -> String {
        let mut out = String::from("{");
        json::field(&mut out, "id", &self.id.to_string());
        json::field(&mut out, "t", &self.epoch_ms.to_string());
        json::field(&mut out, "level", &json::quoted(self.level.as_str()));
        json::field(&mut out, "source", &json::quoted(self.source.as_str()));
        json::field(&mut out, "msg", &json::quoted(&self.message));
        json::field(
            &mut out,
            "code",
            &json::opt_str_json(self.code),
        );
        out.push('}');
        out
    }

    pub fn csv_row(&self) -> String {
        format!(
            "{},{},{},{},{}\n",
            clock::iso8601_utc(self.epoch_ms),
            self.level.as_str(),
            self.source.as_str(),
            csv_escape(&self.message),
            csv_escape(self.code.unwrap_or(""))
        )
    }
}

pub fn csv_escape(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_string()
    }
}

/// Append-only human-readable session log.
pub struct SessionLog {
    file: File,
    path: PathBuf,
    started_ms: u64,
}

impl SessionLog {
    pub fn create(dir: &Path, started_epoch_ms: u64) -> io::Result<Self> {
        std::fs::create_dir_all(dir)?;
        let stamp = clock::iso8601_utc(started_epoch_ms)
            .replace(['-', ':'], "")
            .replace('T', "-");
        let stamp = stamp.trim_end_matches('Z').split('.').next().unwrap_or("session");
        let path = dir.join(format!("session-{stamp}.log"));
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        let mut log = SessionLog {
            file,
            path,
            started_ms: started_epoch_ms,
        };
        let _ = writeln!(
            log.file,
            "# GSE ground station session {}\n# started {} ({} ms since epoch)",
            env!("CARGO_PKG_VERSION"),
            clock::iso8601_utc(started_epoch_ms),
            started_epoch_ms
        );
        Ok(log)
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn write(&mut self, entry: &LogEntry) {
        let uptime = entry.epoch_ms.saturating_sub(self.started_ms);
        let code = entry.code.map(|c| format!(" ({c})")).unwrap_or_default();
        let line = format!(
            "{}  {:<4}  [{:<4}] +{:<9} {}{}\n",
            clock::iso8601_utc(entry.epoch_ms),
            entry.level.label(),
            entry.source.label(),
            format_args!("{}ms", uptime),
            entry.message,
            code
        );
        let _ = self.file.write_all(line.as_bytes());
        let _ = self.file.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_and_csv_escape() {
        let entry = LogEntry {
            id: 7,
            epoch_ms: 0,
            level: Level::Warn,
            source: Source::Vehicle,
            message: "hello, \"world\"\nline".to_string(),
            code: Some("fill_auto_off"),
        };
        let parsed = json::parse(&entry.to_json()).unwrap();
        assert_eq!(parsed.get("level").unwrap().as_str(), Some("warn"));
        assert_eq!(parsed.get("code").unwrap().as_str(), Some("fill_auto_off"));
        let csv = entry.csv_row();
        assert!(csv.starts_with("1970-01-01T00:00:00.000Z,warn,vehicle,\"hello,"));
        assert!(csv.contains("\"\"world\"\""));
    }

    #[test]
    fn session_log_roundtrip() {
        let dir = std::env::temp_dir().join(format!(
            "gse-log-test-{}-{}",
            std::process::id(),
            clock::unix_ms()
        ));
        let mut log = SessionLog::create(&dir, clock::unix_ms()).unwrap();
        let entry = LogEntry {
            id: 1,
            epoch_ms: clock::unix_ms(),
            level: Level::Ok,
            source: Source::App,
            message: "test entry".to_string(),
            code: Some("boot"),
        };
        log.write(&entry);
        let text = std::fs::read_to_string(log.path()).unwrap();
        assert!(text.contains("test entry"));
        assert!(text.contains("(boot)"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn levels_sort_by_severity() {
        assert!(Level::Info < Level::Warn);
        assert!(Level::Warn < Level::Error);
    }
}
