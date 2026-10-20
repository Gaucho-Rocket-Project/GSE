//! Embedded single-page operator console.
//!
//! The GUI is served by the Rust process and opened in the system browser.
//! This keeps the application dependency-free and identical on Windows, Linux
//! and macOS while all serial, protocol and safety logic stays in Rust.

pub const INDEX_HTML: &str = include_str!("../web/index.html");
pub const APP_CSS: &str = include_str!("../web/app.css");
pub const APP_JS: &str = include_str!("../web/app.js");
pub const FAVICON: &str = include_str!("../web/favicon.svg");

pub fn index_html(token: &str) -> String {
    INDEX_HTML.replace("__GSE_TOKEN__", token)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn assets_are_embedded() {
        assert!(INDEX_HTML.contains("__GSE_TOKEN__"));
        assert!(APP_CSS.contains("--bg"));
        assert!(APP_JS.contains("EventSource"));
        assert!(FAVICON.contains("<svg"));
    }

    #[test]
    fn token_is_injected() {
        let html = index_html("deadbeef");
        assert!(html.contains("deadbeef"));
        assert!(!html.contains("__GSE_TOKEN__"));
    }
}
