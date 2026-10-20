//! Operator settings, persisted as JSON.

use std::path::Path;

use gse_proto::{MAX_FILL_SECONDS, MIN_FILL_SECONDS};

use crate::json::{self, Value};

/// Action ids used by the keyboard shortcut system.
pub const SHORTCUT_ACTIONS: &[(&str, &str)] = &[
    ("fill_toggle", "FILL ON / OFF"),
    ("srv_toggle", "SRV ON / OFF"),
    ("ign_toggle", "IGN PWR / SAFE"),
    ("estop", "All safe (E-STOP)"),
    ("reset_fill", "Reset fill sequence"),
    ("clear_faults", "Clear latched faults"),
];

pub const DEFAULT_SHORTCUTS: &[(&str, &str)] = &[
    ("fill_toggle", "F"),
    ("srv_toggle", "S"),
    ("ign_toggle", "I"),
    ("estop", "E"),
    ("reset_fill", "R"),
    ("clear_faults", "Shift+C"),
];

pub const BAUDS: &[u32] = &[9_600, 19_200, 38_400, 57_600, 115_200, 230_400];

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    pub fill_seconds: u16,
    /// `"auto"` for protocol-based port detection, otherwise a device path.
    pub port: String,
    pub baud: u32,
    pub confirm_ign: bool,
    pub sound: bool,
    pub host: String,
    pub http_port: u16,
    pub shortcuts: Vec<(String, String)>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            fill_seconds: gse_proto::DEFAULT_FILL_SECONDS,
            port: "auto".to_string(),
            baud: gse_proto::USB_BAUD,
            confirm_ign: true,
            sound: true,
            host: "127.0.0.1".to_string(),
            http_port: 8765,
            shortcuts: DEFAULT_SHORTCUTS
                .iter()
                .map(|(a, k)| (a.to_string(), k.to_string()))
                .collect(),
        }
    }
}

impl Config {
    pub fn shortcut(&self, action: &str) -> String {
        self.shortcuts
            .iter()
            .find(|(a, _)| a == action)
            .map(|(_, k)| k.clone())
            .unwrap_or_default()
    }

    pub fn set_shortcut(&mut self, action: &str, combo: &str) -> Result<(), String> {
        if !SHORTCUT_ACTIONS.iter().any(|(a, _)| *a == action) {
            return Err(format!("unknown action '{action}'"));
        }
        let combo = combo.trim();
        if combo.is_empty() || combo.len() > 32 {
            return Err("key binding must be 1-32 characters".to_string());
        }
        if combo.chars().any(|c| c.is_control()) {
            return Err("key binding contains control characters".to_string());
        }
        if let Some((other, _)) = self
            .shortcuts
            .iter()
            .find(|(a, k)| a != action && k.eq_ignore_ascii_case(combo))
        {
            return Err(format!("'{combo}' is already bound to {other}"));
        }
        if let Some(slot) = self.shortcuts.iter_mut().find(|(a, _)| a == action) {
            slot.1 = combo.to_string();
        } else {
            self.shortcuts.push((action.to_string(), combo.to_string()));
        }
        Ok(())
    }

    pub fn reset_shortcuts(&mut self) {
        self.shortcuts = DEFAULT_SHORTCUTS
            .iter()
            .map(|(a, k)| (a.to_string(), k.to_string()))
            .collect();
    }

    /// Clamp/repair values that came from disk or the browser.
    pub fn sanitize(&mut self) {
        self.fill_seconds = self
            .fill_seconds
            .clamp(MIN_FILL_SECONDS, MAX_FILL_SECONDS);
        if !BAUDS.contains(&self.baud) {
            self.baud = gse_proto::USB_BAUD;
        }
        self.port = self.port.trim().to_string();
        if self.port.is_empty() {
            self.port = "auto".to_string();
        }
        self.host = self.host.trim().to_string();
        if self.host.is_empty() {
            self.host = "127.0.0.1".to_string();
        }
        if self.http_port < 1024 {
            self.http_port = 8765;
        }
        let mut seen = Vec::new();
        self.shortcuts.retain(|(a, k)| {
            if !SHORTCUT_ACTIONS.iter().any(|(id, _)| id == a) {
                return false;
            }
            if k.trim().is_empty() || seen.iter().any(|s: &String| s.eq_ignore_ascii_case(k)) {
                return false;
            }
            seen.push(k.clone());
            true
        });
        for (action, default) in DEFAULT_SHORTCUTS {
            if !self.shortcuts.iter().any(|(a, _)| a == action) {
                self.shortcuts
                    .push((action.to_string(), default.to_string()));
            }
        }
    }

    pub fn to_json(&self) -> String {
        let mut out = String::from("{");
        json::field(&mut out, "fill_seconds", &self.fill_seconds.to_string());
        json::field(&mut out, "port", &json::quoted(&self.port));
        json::field(&mut out, "baud", &self.baud.to_string());
        json::field(&mut out, "confirm_ign", json::bool_json(self.confirm_ign));
        json::field(&mut out, "sound", json::bool_json(self.sound));
        json::field(&mut out, "host", &json::quoted(&self.host));
        json::field(&mut out, "http_port", &self.http_port.to_string());
        let mut sc = String::from("{");
        for (action, combo) in &self.shortcuts {
            json::field(&mut sc, action, &json::quoted(combo));
        }
        sc.push('}');
        json::field(&mut out, "shortcuts", &sc);
        out.push('}');
        out
    }

    pub fn from_json(value: &Value) -> Result<Config, String> {
        let mut cfg = Config::default();
        if let Some(v) = value.get("fill_seconds").and_then(Value::as_i64) {
            cfg.fill_seconds = v.clamp(MIN_FILL_SECONDS as i64, MAX_FILL_SECONDS as i64) as u16;
        }
        if let Some(v) = value.get("port").and_then(Value::as_str) {
            cfg.port = v.to_string();
        }
        if let Some(v) = value.get("baud").and_then(Value::as_i64) {
            cfg.baud = v as u32;
        }
        if let Some(v) = value.get("confirm_ign").and_then(Value::as_bool) {
            cfg.confirm_ign = v;
        }
        if let Some(v) = value.get("sound").and_then(Value::as_bool) {
            cfg.sound = v;
        }
        if let Some(v) = value.get("host").and_then(Value::as_str) {
            cfg.host = v.to_string();
        }
        if let Some(v) = value.get("http_port").and_then(Value::as_i64) {
            cfg.http_port = v as u16;
        }
        if let Some(v) = value.get("shortcuts") {
            for (action, combo) in v.entries() {
                if let Some(combo) = combo.as_str() {
                    let _ = cfg.set_shortcut(action, combo);
                }
            }
        }
        cfg.sanitize();
        Ok(cfg)
    }

    pub fn load(path: &Path) -> (Config, Option<String>) {
        match std::fs::read_to_string(path) {
            Ok(text) => match json::parse(&text) {
                Ok(value) => match Config::from_json(&value) {
                    Ok(cfg) => (cfg, None),
                    Err(e) => (
                        Config::default(),
                        Some(format!("invalid config {}: {e}", path.display())),
                    ),
                },
                Err(e) => (
                    Config::default(),
                    Some(format!("cannot parse {}: {e}", path.display())),
                ),
            },
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => (Config::default(), None),
            Err(e) => (
                Config::default(),
                Some(format!("cannot read {}: {e}", path.display())),
            ),
        }
    }

    pub fn save(&self, path: &Path) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let tmp = path.with_extension("json.tmp");
        std::fs::write(&tmp, self.to_pretty_json())?;
        std::fs::rename(&tmp, path)?;
        Ok(())
    }

    pub fn to_pretty_json(&self) -> String {
        let mut out = String::from("{\n");
        out.push_str(&format!("  \"fill_seconds\": {},\n", self.fill_seconds));
        out.push_str(&format!("  \"port\": {},\n", json::quoted(&self.port)));
        out.push_str(&format!("  \"baud\": {},\n", self.baud));
        out.push_str(&format!("  \"confirm_ign\": {},\n", self.confirm_ign));
        out.push_str(&format!("  \"sound\": {},\n", self.sound));
        out.push_str(&format!("  \"host\": {},\n", json::quoted(&self.host)));
        out.push_str(&format!("  \"http_port\": {},\n", self.http_port));
        out.push_str("  \"shortcuts\": {\n");
        for (i, (action, combo)) in self.shortcuts.iter().enumerate() {
            out.push_str(&format!(
                "    {}: {}{}\n",
                json::quoted(action),
                json::quoted(combo),
                if i + 1 == self.shortcuts.len() { "" } else { "," }
            ));
        }
        out.push_str("  }\n}\n");
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_roundtrip() {
        let mut cfg = Config::default();
        cfg.fill_seconds = 42;
        cfg.port = "/dev/ttyACM0".into();
        cfg.confirm_ign = false;
        cfg.set_shortcut("fill_toggle", "Ctrl+Shift+F").unwrap();
        let text = cfg.to_json();
        let parsed = Config::from_json(&json::parse(&text).unwrap()).unwrap();
        assert_eq!(parsed, cfg);
    }

    #[test]
    fn clamping_and_repair() {
        let value = json::parse(
            r#"{"fill_seconds":9999,"baud":1234,"port":"  ","host":"","http_port":80,"shortcuts":{"fill_toggle":"","srv_toggle":"G"}}"#,
        )
        .unwrap();
        let cfg = Config::from_json(&value).unwrap();
        assert_eq!(cfg.fill_seconds, MAX_FILL_SECONDS);
        assert_eq!(cfg.baud, 115_200);
        assert_eq!(cfg.port, "auto");
        assert_eq!(cfg.host, "127.0.0.1");
        assert_eq!(cfg.http_port, 8765);
        // Empty binding dropped and replaced with the default.
        assert_eq!(cfg.shortcut("fill_toggle"), "F");
        assert_eq!(cfg.shortcut("srv_toggle"), "G");
    }

    #[test]
    fn duplicate_bindings_rejected() {
        let mut cfg = Config::default();
        assert!(cfg.set_shortcut("fill_toggle", "S").is_err());
        assert!(cfg.set_shortcut("fill_toggle", "Ctrl+F").is_ok());
        assert!(cfg.set_shortcut("bogus", "X").is_err());
        assert!(cfg.set_shortcut("fill_toggle", "").is_err());
    }

    #[test]
    fn reset_shortcuts_restores_defaults() {
        let mut cfg = Config::default();
        cfg.set_shortcut("fill_toggle", "Ctrl+F").unwrap();
        cfg.reset_shortcuts();
        assert_eq!(cfg.shortcut("fill_toggle"), "F");
    }

    #[test]
    fn pretty_json_is_valid() {
        let cfg = Config::default();
        let parsed = json::parse(&cfg.to_pretty_json()).unwrap();
        assert_eq!(
            parsed.get("fill_seconds").unwrap().as_u64(),
            Some(60)
        );
    }
}
