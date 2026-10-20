//! Platform-specific configuration and log directories (no external crates).

use std::path::PathBuf;

fn env(name: &str) -> Option<String> {
    std::env::var(name).ok().filter(|v| !v.is_empty())
}

fn home() -> Option<PathBuf> {
    #[cfg(windows)]
    {
        env("USERPROFILE").map(PathBuf::from)
    }
    #[cfg(not(windows))]
    {
        env("HOME").map(PathBuf::from)
    }
}

/// `~/.config/gse`, `~/Library/Application Support/GSE`, `%APPDATA%\GSE`.
pub fn config_dir() -> PathBuf {
    #[cfg(windows)]
    {
        if let Some(appdata) = env("APPDATA") {
            return PathBuf::from(appdata).join("GSE");
        }
        home().unwrap_or_else(|| PathBuf::from(".")).join("GSE")
    }
    #[cfg(target_os = "macos")]
    {
        home()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("Library")
            .join("Application Support")
            .join("GSE")
    }
    #[cfg(all(unix, not(target_os = "macos")))]
    {
        if let Some(xdg) = env("XDG_CONFIG_HOME") {
            return PathBuf::from(xdg).join("gse");
        }
        home()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(".config")
            .join("gse")
    }
}

pub fn default_config_path() -> PathBuf {
    config_dir().join("config.json")
}

/// Per-user log directory.
pub fn logs_dir() -> PathBuf {
    #[cfg(target_os = "macos")]
    {
        home()
            .unwrap_or_else(|| PathBuf::from("."))
            .join("Library")
            .join("Logs")
            .join("GSE")
    }
    #[cfg(all(unix, not(target_os = "macos")))]
    {
        if let Some(state) = env("XDG_STATE_HOME") {
            return PathBuf::from(state).join("gse").join("logs");
        }
        config_dir().join("logs")
    }
    #[cfg(windows)]
    {
        config_dir().join("logs")
    }
}

/// Directory containing the running executable (used to find `gse-sim`).
pub fn exe_dir() -> PathBuf {
    std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|p| p.to_path_buf()))
        .unwrap_or_else(|| PathBuf::from("."))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn paths_are_absolute_or_relative_but_nonempty() {
        assert!(!config_dir().as_os_str().is_empty());
        assert!(default_config_path().ends_with("config.json"));
        assert!(!logs_dir().as_os_str().is_empty());
        assert!(!exe_dir().as_os_str().is_empty());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn linux_xdg_layout() {
        if std::env::var("XDG_CONFIG_HOME").is_err() {
            let dir = config_dir();
            assert!(dir.ends_with("gse"));
        }
    }
}
