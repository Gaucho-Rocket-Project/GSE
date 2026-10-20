//! Minimal HTTP/1.1 server for the operator console and JSON API.
//!
//! Only the pieces needed by this application are implemented: a request line,
//! headers, `Content-Length` bodies, static assets, JSON endpoints and
//! Server-Sent Events. No external crates, no TLS (the listener is local by
//! default).

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::sync::mpsc::Sender;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crate::broadcaster::EventMsg;
use crate::clock;
use crate::config::{Config, BAUDS};
use crate::engine::{ConfigPatch, Request, UserAction};
use crate::json::{self, Value};
use crate::serial;
use crate::state::Shared;
use crate::ui;

const MAX_HEADER_BYTES: usize = 64 * 1024;
const MAX_BODY_BYTES: usize = 1024 * 1024;

pub struct HttpRequest {
    pub method: String,
    pub path: String,
    pub query: Vec<(String, String)>,
    pub headers: Vec<(String, String)>,
    pub body: Vec<u8>,
}

impl HttpRequest {
    pub fn header(&self, name: &str) -> Option<&str> {
        self.headers
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(name))
            .map(|(_, v)| v.as_str())
    }

    pub fn query_value(&self, name: &str) -> Option<&str> {
        self.query
            .iter()
            .find(|(k, _)| k == name)
            .map(|(_, v)| v.as_str())
    }

    pub fn json_body(&self) -> Result<Value, String> {
        let text = std::str::from_utf8(&self.body).map_err(|_| "body is not UTF-8".to_string())?;
        if text.trim().is_empty() {
            return Ok(Value::Object(Vec::new()));
        }
        json::parse(text)
    }
}

/// Bind a listener, trying a handful of ports if the requested one is busy.
pub fn bind(host: &str, port: u16) -> io::Result<(TcpListener, SocketAddr)> {
    let mut last_err = None;
    let attempts = if port == 0 { 1 } else { 20 };
    for offset in 0..attempts {
        let candidate = if port == 0 {
            0
        } else {
            port.saturating_add(offset)
        };
        let addr = format!("{host}:{candidate}");
        let resolved: Vec<SocketAddr> = addr.to_socket_addrs()?.collect();
        for sock in resolved {
            match TcpListener::bind(sock) {
                Ok(listener) => {
                    listener.set_nonblocking(false)?;
                    let local = listener.local_addr()?;
                    return Ok((listener, local));
                }
                Err(e) => last_err = Some(e),
            }
        }
    }
    Err(last_err
        .unwrap_or_else(|| io::Error::new(io::ErrorKind::AddrInUse, "could not bind any port")))
}

pub fn url_for(addr: SocketAddr, host: &str) -> String {
    let display_host = if host == "0.0.0.0" || host == "::" {
        "127.0.0.1"
    } else {
        host
    };
    if addr.port() == 80 {
        format!("http://{display_host}/")
    } else {
        format!("http://{display_host}:{}/", addr.port())
    }
}

/// Accept loop; call this on a dedicated thread.
pub fn serve(listener: TcpListener, shared: Arc<Shared>, requests: Sender<Request>) {
    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                let shared = Arc::clone(&shared);
                let requests = requests.clone();
                let _ = thread::Builder::new()
                    .name("gse-http-conn".to_string())
                    .spawn(move || {
                        if let Err(e) = handle_connection(stream, &shared, &requests) {
                            if shared.verbose {
                                eprintln!("[gse] HTTP connection error: {e}");
                            }
                        }
                    });
            }
            Err(e) => {
                if shared.verbose {
                    eprintln!("[gse] accept error: {e}");
                }
                thread::sleep(Duration::from_millis(50));
            }
        }
    }
}

fn handle_connection(
    mut stream: TcpStream,
    shared: &Arc<Shared>,
    requests: &Sender<Request>,
) -> io::Result<()> {
    let token = shared.token.as_str();
    let verbose = shared.verbose;
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    stream.set_write_timeout(Some(Duration::from_secs(10)))?;
    stream.set_nodelay(true).ok();
    let request = match read_request(&mut stream) {
        Ok(req) => req,
        Err(e) => {
            let _ = write_response(
                &mut stream,
                400,
                &[("Content-Type", "text/plain".into())],
                e.to_string().as_bytes(),
                false,
            );
            return Ok(());
        }
    };

    if verbose {
        eprintln!("[gse] {} {}", request.method, request.path);
    }

    let path = request.path.as_str();
    match (request.method.as_str(), path) {
        ("GET", "/") | ("GET", "/index.html") => {
            let html = ui::index_html(token);
            write_response(
                &mut stream,
                200,
                &[
                    ("Content-Type", "text/html; charset=utf-8".into()),
                    ("Cache-Control", "no-store".into()),
                    ("X-Content-Type-Options", "nosniff".into()),
                    (
                        "Content-Security-Policy",
                        "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; base-uri 'none'; form-action 'none'".into(),
                    ),
                ],
                html.as_bytes(),
                false,
            )
        }
        ("GET", "/app.css") => write_response(
            &mut stream,
            200,
            &[
                ("Content-Type", "text/css; charset=utf-8".into()),
                ("Cache-Control", "max-age=300".into()),
            ],
            ui::APP_CSS.as_bytes(),
            false,
        ),
        ("GET", "/app.js") => write_response(
            &mut stream,
            200,
            &[
                ("Content-Type", "text/javascript; charset=utf-8".into()),
                ("Cache-Control", "max-age=300".into()),
            ],
            ui::APP_JS.as_bytes(),
            false,
        ),
        ("GET", "/favicon.svg") => write_response(
            &mut stream,
            200,
            &[
                ("Content-Type", "image/svg+xml".into()),
                ("Cache-Control", "max-age=86400".into()),
            ],
            ui::FAVICON.as_bytes(),
            false,
        ),
        ("GET", "/api/health") => {
            let body = format!(
                "{{\"ok\":true,\"subscribers\":{}}}",
                shared.events.subscriber_count()
            );
            write_json(&mut stream, 200, &body)
        }
        ("GET", "/api/state") => {
            let body = shared.snapshot();
            write_json(&mut stream, 200, &body)
        }
        ("GET", "/api/ports") => {
            let ports = serial::list_ports();
            let mut out = String::from("{\"ports\":[");
            for (i, port) in ports.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                let mut item = String::from("{");
                json::field(&mut item, "path", &json::quoted(&port.path));
                json::field(&mut item, "label", &json::quoted(&port.label));
                json::field(&mut item, "kind", &json::quoted(port.kind));
                item.push('}');
                out.push_str(&item);
            }
            out.push_str("],\"selected\":");
            let selected = shared.lock().config.port.clone();
            out.push_str(&json::quoted(&selected));
            out.push('}');
            write_json(&mut stream, 200, &out)
        }
        ("GET", "/api/log") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            let limit = request
                .query_value("limit")
                .and_then(|v| v.parse::<usize>().ok())
                .unwrap_or(300)
                .min(1000);
            let entries: Vec<String> = {
                let st = shared.lock();
                st.log
                    .iter()
                    .rev()
                    .take(limit)
                    .map(|e| e.to_json())
                    .collect::<Vec<_>>()
                    .into_iter()
                    .rev()
                    .collect()
            };
            let body = format!("{{\"entries\":[{}]}}", entries.join(","));
            write_json(&mut stream, 200, &body)
        }
        ("GET", "/api/log.csv") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            let mut body = String::from("time,level,source,message,code\n");
            {
                let st = shared.lock();
                for entry in &st.log {
                    body.push_str(&entry.csv_row());
                }
            }
            write_response(
                &mut stream,
                200,
                &[
                    ("Content-Type", "text/csv; charset=utf-8".into()),
                    (
                        "Content-Disposition",
                        "attachment; filename=\"gse-session.csv\"".into(),
                    ),
                    ("Cache-Control", "no-store".into()),
                ],
                body.as_bytes(),
                false,
            )
        }
        ("POST", "/api/command") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            match parse_action(&request) {
                Ok(action) => {
                    let _ = requests.send(Request::Action(action));
                    write_json(&mut stream, 200, "{\"ok\":true}")
                }
                Err(e) => write_json(
                    &mut stream,
                    400,
                    &format!("{{\"error\":{}}}", json::quoted(&e)),
                ),
            }
        }
        ("POST", "/api/config") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            match parse_config_patch(&request) {
                Ok(patch) => {
                    let _ = requests.send(Request::SetConfig(patch));
                    write_json(&mut stream, 200, "{\"ok\":true}")
                }
                Err(e) => write_json(
                    &mut stream,
                    400,
                    &format!("{{\"error\":{}}}", json::quoted(&e)),
                ),
            }
        }
        ("POST", "/api/shortcuts") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            match parse_shortcut(&request, shared) {
                Ok(Some(Request::ResetShortcuts)) => {
                    let _ = requests.send(Request::ResetShortcuts);
                    write_json(&mut stream, 200, "{\"ok\":true}")
                }
                Ok(Some(req)) => {
                    let _ = requests.send(req);
                    write_json(&mut stream, 200, "{\"ok\":true}")
                }
                Ok(None) => write_json(&mut stream, 200, "{\"ok\":true}"),
                Err(e) => write_json(
                    &mut stream,
                    400,
                    &format!("{{\"error\":{}}}", json::quoted(&e)),
                ),
            }
        }
        ("POST", "/api/connect") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            let _ = requests.send(Request::Connect);
            write_json(&mut stream, 200, "{\"ok\":true}")
        }
        ("POST", "/api/disconnect") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            let _ = requests.send(Request::Disconnect);
            write_json(&mut stream, 200, "{\"ok\":true}")
        }
        ("POST", "/api/shutdown") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            write_json(&mut stream, 200, "{\"ok\":true,\"stopping\":true}")?;
            let _ = requests.send(Request::Shutdown);
            crate::signals::request();
            Ok(())
        }
        ("POST", "/api/log/clear") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            let _ = requests.send(Request::ClearLog);
            write_json(&mut stream, 200, "{\"ok\":true}")
        }
        ("GET", "/api/events") => {
            if !authorized(&request, token) {
                return forbidden(&mut stream);
            }
            handle_sse(stream, shared)
        }
        _ => {
            if path.starts_with("/api/") {
                write_json(&mut stream, 404, "{\"error\":\"not found\"}")
            } else {
                write_response(
                    &mut stream,
                    404,
                    &[("Content-Type", "text/plain; charset=utf-8".into())],
                    b"not found\n",
                    false,
                )
            }
        }
    }
}

fn forbidden(stream: &mut TcpStream) -> io::Result<()> {
    write_json(
        stream,
        403,
        "{\"error\":\"invalid or missing session token\"}",
    )
}

fn authorized(request: &HttpRequest, token: &str) -> bool {
    if let Some(value) = request.header("x-gse-token") {
        if value == token {
            return true;
        }
    }
    matches!(request.query_value("token"), Some(v) if v == token)
}

fn read_request(stream: &mut TcpStream) -> io::Result<HttpRequest> {
    let mut data = Vec::new();
    let mut buf = [0u8; 4096];
    let header_end;
    loop {
        let n = stream.read(&mut buf)?;
        if n == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "client closed before request",
            ));
        }
        data.extend_from_slice(&buf[..n]);
        if let Some(pos) = find_subsequence(&data, b"\r\n\r\n") {
            header_end = pos + 4;
            break;
        }
        if data.len() > MAX_HEADER_BYTES {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "request headers too large",
            ));
        }
    }

    let head = std::str::from_utf8(&data[..header_end])
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "headers are not valid UTF-8"))?;
    let mut lines = head.split("\r\n");
    let request_line = lines
        .next()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "missing request line"))?;
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let target = parts.next().unwrap_or("/").to_string();
    if method.is_empty() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "missing method"));
    }
    let (path, query_string) = match target.split_once('?') {
        Some((p, q)) => (p.to_string(), q.to_string()),
        None => (target, String::new()),
    };
    let query = parse_query(&query_string);
    let mut headers = Vec::new();
    for line in lines {
        if line.is_empty() {
            continue;
        }
        if let Some((key, value)) = line.split_once(':') {
            headers.push((key.trim().to_string(), value.trim().to_string()));
        }
    }

    let content_length = headers
        .iter()
        .find(|(k, _)| k.eq_ignore_ascii_case("content-length"))
        .and_then(|(_, v)| v.parse::<usize>().ok())
        .unwrap_or(0);
    if content_length > MAX_BODY_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "request body too large",
        ));
    }
    let mut body = data[header_end..].to_vec();
    while body.len() < content_length {
        let n = stream.read(&mut buf)?;
        if n == 0 {
            break;
        }
        body.extend_from_slice(&buf[..n]);
    }
    body.truncate(content_length);

    Ok(HttpRequest {
        method,
        path,
        query,
        headers,
        body,
    })
}

fn parse_query(query: &str) -> Vec<(String, String)> {
    query
        .split('&')
        .filter(|p| !p.is_empty())
        .map(|pair| match pair.split_once('=') {
            Some((k, v)) => (percent_decode(k), percent_decode(v)),
            None => (percent_decode(pair), String::new()),
        })
        .collect()
}

fn percent_decode(input: &str) -> String {
    let bytes = input.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'%' if i + 2 < bytes.len() => {
                let hi = (bytes[i + 1] as char).to_digit(16);
                let lo = (bytes[i + 2] as char).to_digit(16);
                if let (Some(hi), Some(lo)) = (hi, lo) {
                    out.push((hi * 16 + lo) as u8);
                    i += 3;
                    continue;
                }
                out.push(b'%');
                i += 1;
            }
            b'+' => {
                out.push(b' ');
                i += 1;
            }
            b => {
                out.push(b);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn find_subsequence(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || haystack.len() < needle.len() {
        return None;
    }
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
}

fn parse_action(request: &HttpRequest) -> Result<UserAction, String> {
    let body = request.json_body()?;
    let action = body
        .get("action")
        .and_then(Value::as_str)
        .ok_or_else(|| "missing 'action'".to_string())?;
    UserAction::from_id(action).ok_or_else(|| format!("unknown action '{action}'"))
}

fn parse_config_patch(request: &HttpRequest) -> Result<ConfigPatch, String> {
    let body = request.json_body()?;
    let mut patch = ConfigPatch::default();
    if let Some(v) = body.get("fill_seconds") {
        let seconds = v
            .as_i64()
            .ok_or_else(|| "fill_seconds must be a number".to_string())?;
        if seconds < gse_proto::MIN_FILL_SECONDS as i64
            || seconds > gse_proto::MAX_FILL_SECONDS as i64
        {
            return Err(format!(
                "fill_seconds must be between {} and {}",
                gse_proto::MIN_FILL_SECONDS,
                gse_proto::MAX_FILL_SECONDS
            ));
        }
        patch.fill_seconds = Some(seconds as u16);
    }
    if let Some(v) = body.get("port") {
        let port = v
            .as_str()
            .ok_or_else(|| "port must be a string".to_string())?;
        if port.trim().is_empty() {
            return Err("port must not be empty".to_string());
        }
        patch.port = Some(port.trim().to_string());
    }
    if let Some(v) = body.get("baud") {
        let baud = v
            .as_i64()
            .ok_or_else(|| "baud must be a number".to_string())? as u32;
        if !BAUDS.contains(&baud) {
            return Err(format!("unsupported baud rate {baud}"));
        }
        patch.baud = Some(baud);
    }
    if let Some(v) = body.get("confirm_ign") {
        patch.confirm_ign = Some(
            v.as_bool()
                .ok_or_else(|| "confirm_ign must be a boolean".to_string())?,
        );
    }
    if let Some(v) = body.get("sound") {
        patch.sound = Some(
            v.as_bool()
                .ok_or_else(|| "sound must be a boolean".to_string())?,
        );
    }
    Ok(patch)
}

fn parse_shortcut(request: &HttpRequest, shared: &Arc<Shared>) -> Result<Option<Request>, String> {
    let body = request.json_body()?;
    if body.get("reset").and_then(Value::as_bool).unwrap_or(false) {
        return Ok(Some(Request::ResetShortcuts));
    }
    let action = body
        .get("action")
        .and_then(Value::as_str)
        .ok_or_else(|| "missing 'action'".to_string())?;
    let combo = body
        .get("combo")
        .and_then(Value::as_str)
        .ok_or_else(|| "missing 'combo'".to_string())?;
    // Validate against a copy so the browser gets an immediate 400 on
    // conflicts instead of an asynchronous log line.
    let mut cfg: Config = shared.lock().config.clone();
    cfg.set_shortcut(action, combo)?;
    Ok(Some(Request::SetShortcut {
        action: action.to_string(),
        combo: combo.trim().to_string(),
    }))
}

fn write_json(stream: &mut TcpStream, status: u16, body: &str) -> io::Result<()> {
    write_response(
        stream,
        status,
        &[
            ("Content-Type", "application/json; charset=utf-8".into()),
            ("Cache-Control", "no-store".into()),
        ],
        body.as_bytes(),
        false,
    )
}

fn status_text(status: u16) -> &'static str {
    match status {
        200 => "OK",
        400 => "Bad Request",
        403 => "Forbidden",
        404 => "Not Found",
        405 => "Method Not Allowed",
        413 => "Payload Too Large",
        500 => "Internal Server Error",
        _ => "OK",
    }
}

fn write_response(
    stream: &mut TcpStream,
    status: u16,
    headers: &[(&str, String)],
    body: &[u8],
    keep_alive: bool,
) -> io::Result<()> {
    let mut head = format!(
        "HTTP/1.1 {} {}\r\nContent-Length: {}\r\nConnection: {}\r\n",
        status,
        status_text(status),
        body.len(),
        if keep_alive { "keep-alive" } else { "close" }
    );
    for (key, value) in headers {
        head.push_str(key);
        head.push_str(": ");
        head.push_str(value);
        head.push_str("\r\n");
    }
    head.push_str("\r\n");
    stream.write_all(head.as_bytes())?;
    stream.write_all(body)?;
    stream.flush()
}

fn handle_sse(mut stream: TcpStream, shared: &Arc<Shared>) -> io::Result<()> {
    stream.set_write_timeout(Some(Duration::from_secs(30)))?;
    stream.set_read_timeout(None)?;
    let head = "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-cache, no-store\r\nConnection: keep-alive\r\nX-Accel-Buffering: no\r\n\r\nretry: 1000\n\n";
    stream.write_all(head.as_bytes())?;
    let (rx, initial) = shared.events.subscribe();
    let mut payload = String::with_capacity(initial.len() + 16);
    payload.push_str("event: state\ndata: ");
    payload.push_str(&initial);
    payload.push_str("\n\n");
    if stream.write_all(payload.as_bytes()).is_err() {
        return Ok(());
    }
    loop {
        match rx.recv_timeout(Duration::from_secs(15)) {
            Ok(EventMsg::State(json)) => {
                let frame = format!("event: state\ndata: {json}\n\n");
                if stream.write_all(frame.as_bytes()).is_err() {
                    break;
                }
            }
            Ok(EventMsg::Log(json)) => {
                let frame = format!("event: log\ndata: {json}\n\n");
                if stream.write_all(frame.as_bytes()).is_err() {
                    break;
                }
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                if stream
                    .write_all(
                        format!(": keepalive {:?}\n\n", clock::iso8601_utc(clock::unix_ms()))
                            .as_bytes(),
                    )
                    .is_err()
                {
                    break;
                }
            }
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(method: &str, target: &str) -> HttpRequest {
        let (path, query) = match target.split_once('?') {
            Some((p, q)) => (p.to_string(), parse_query(q)),
            None => (target.to_string(), Vec::new()),
        };
        HttpRequest {
            method: method.to_string(),
            path,
            query,
            headers: Vec::new(),
            body: Vec::new(),
        }
    }

    #[test]
    fn query_parsing() {
        let r = request("GET", "/api/log?limit=50&token=abc%20def");
        assert_eq!(r.query_value("limit"), Some("50"));
        assert_eq!(r.query_value("token"), Some("abc def"));
    }

    #[test]
    fn subsequence_search() {
        assert_eq!(find_subsequence(b"abc\r\n\r\ndef", b"\r\n\r\n"), Some(3));
        assert_eq!(find_subsequence(b"abc", b"\r\n\r\n"), None);
    }

    #[test]
    fn action_parsing() {
        let mut r = request("POST", "/api/command");
        r.body = br#"{"action":"fill_on"}"#.to_vec();
        assert_eq!(parse_action(&r).unwrap(), UserAction::FillOn);
        r.body = br#"{"action":"reset_controls"}"#.to_vec();
        assert_eq!(parse_action(&r).unwrap(), UserAction::ResetControls);
        r.body = br#"{"action":"nope"}"#.to_vec();
        assert!(parse_action(&r).is_err());
    }

    #[test]
    fn config_validation() {
        let mut r = request("POST", "/api/config");
        r.body = br#"{"fill_seconds":90,"baud":115200,"port":"auto"}"#.to_vec();
        let patch = parse_config_patch(&r).unwrap();
        assert_eq!(patch.fill_seconds, Some(90));
        r.body = br#"{"fill_seconds":9999}"#.to_vec();
        assert!(parse_config_patch(&r).is_err());
        r.body = br#"{"baud":1234}"#.to_vec();
        assert!(parse_config_patch(&r).is_err());
    }

    #[test]
    fn auth_uses_header_or_query() {
        let token = "abc123";
        let mut r = request("POST", "/api/command");
        assert!(!authorized(&r, token));
        r.headers
            .push(("x-gse-token".to_string(), token.to_string()));
        assert!(authorized(&r, token));
        let r = request("GET", "/api/events?token=abc123");
        assert!(authorized(&r, token));
        let r = request("GET", "/api/events?token=nope");
        assert!(!authorized(&r, token));
    }

    #[test]
    fn url_display() {
        let addr: SocketAddr = "127.0.0.1:8765".parse().unwrap();
        assert_eq!(url_for(addr, "127.0.0.1"), "http://127.0.0.1:8765/");
        assert_eq!(url_for(addr, "0.0.0.0"), "http://127.0.0.1:8765/");
    }
}
