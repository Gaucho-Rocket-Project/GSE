//! Ground Support Equipment host application.
//!
//! Talks to the ground NUCLEO-H753ZI board over a USB serial port, serves the
//! operator console on a local HTTP port and drives the vehicle through the
//! LoRa link. No external crates: serial I/O, JSON, HTTP and the UI assets are
//! all part of this workspace.

mod broadcaster;
mod clock;
mod config;
mod engine;
mod http;
mod json;
mod logging;
mod paths;
mod safety;
mod serial;
mod signals;
mod state;
mod transport;
mod ui;

use std::path::PathBuf;
use std::process::Command;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use config::Config;
use engine::Engine;
use logging::{Level, SessionLog, Source};
use state::{AppState, Shared};

const VERSION: &str = env!("CARGO_PKG_VERSION");
const SIM_ADDR: &str = "127.0.0.1:8787";
const SIM_SPEC: &str = "tcp:127.0.0.1:8787";

const USAGE: &str = "\
GSE - ground support equipment for NUCLEO-H753ZI + DX-LR42-24T12D / E28-2G4M27S

USAGE:
    gse-app [OPTIONS]

OPTIONS:
    --sim                 Use the built-in TCP simulator (gse-sim)
    --port <PORT>         Serial port: COM5, /dev/ttyACM0, /dev/tty.usbmodem1234,
                          tcp:host:port, or 'auto' (default)
    --baud <RATE>         USB CDC serial baud rate (default 115200)
    --uart-baud <RATE>    MCU <-> UART radio baud rate (default 9600)
    --radio <MODULE>      Radio module: dx-lr42-24t12d (default) or e28-2g4m27s
    --config <PATH>       Configuration file
    --host <ADDR>         Console bind address (default 127.0.0.1)
    --http-port <PORT>    Console HTTP port (default 8765)
    --no-browser          Do not open a browser window
    --open                Force opening a browser window (even when headless)
    --verbose             Verbose diagnostics on stderr
    -h, --help            Show this help
    -V, --version         Show the version

SAFETY:
    Supports selectable LIVE and TEST control modes. Ignition countdowns hold
    at T-0 with outputs SAFE until the operator explicitly presses L; Shift+L
    aborts an active countdown.
    FILL and SRV are interlocked; neither IGN PWR nor SRV+IGN ON can activate
    without a completed fill sequence. The vehicle firmware independently
    re-checks every command and can always refuse one.

EXAMPLES:
    gse-app                          scan every serial port for the ground board
    gse-app --port /dev/ttyACM0      talk to a specific board
    gse-app --sim                    run against the simulator
";

#[derive(Default)]
struct Options {
    sim: bool,
    port: Option<String>,
    baud: Option<u32>,
    uart_baud: Option<u32>,
    radio: Option<gse_proto::radio::RadioModule>,
    config: Option<PathBuf>,
    host: Option<String>,
    http_port: Option<u16>,
    no_browser: bool,
    force_browser: bool,
    verbose: bool,
    help: bool,
    version: bool,
}

impl Options {
    fn parse<I: IntoIterator<Item = String>>(args: I) -> Result<Options, String> {
        let mut opts = Options::default();
        let mut iter = args.into_iter().peekable();
        while let Some(arg) = iter.next() {
            let (flag, inline) = match arg.split_once('=') {
                Some((f, v)) => (f.to_string(), Some(v.to_string())),
                None => (arg.clone(), None),
            };
            let mut value = |name: &str| -> Result<String, String> {
                if let Some(v) = inline.clone() {
                    return Ok(v);
                }
                iter.next()
                    .ok_or_else(|| format!("{name} requires a value"))
            };
            match flag.as_str() {
                "--sim" => opts.sim = true,
                "--verbose" | "-v" => opts.verbose = true,
                "--no-browser" => opts.no_browser = true,
                "--open" => opts.force_browser = true,
                "--help" | "-h" => opts.help = true,
                "--version" | "-V" => opts.version = true,
                "--port" => opts.port = Some(value("--port")?),
                "--baud" => {
                    let text = value("--baud")?;
                    opts.baud = Some(
                        text.parse::<u32>()
                            .map_err(|_| format!("invalid baud rate '{text}'"))?,
                    );
                }
                "--uart-baud" => {
                    let text = value("--uart-baud")?;
                    opts.uart_baud = Some(
                        text.parse::<u32>()
                            .map_err(|_| format!("invalid UART baud rate '{text}'"))?,
                    );
                }
                "--radio" => {
                    let text = value("--radio")?;
                    opts.radio = Some(gse_proto::radio::RadioModule::parse(&text).ok_or_else(
                        || {
                            format!(
                                "invalid radio module '{text}' (expected dx-lr42-24t12d or e28-2g4m27s)"
                            )
                        },
                    )?);
                }
                "--config" => opts.config = Some(PathBuf::from(value("--config")?)),
                "--host" => opts.host = Some(value("--host")?),
                "--http-port" => {
                    let text = value("--http-port")?;
                    opts.http_port = Some(
                        text.parse::<u16>()
                            .map_err(|_| format!("invalid HTTP port '{text}'"))?,
                    );
                }
                other => {
                    return Err(format!(
                        "unknown option '{other}'\nRun 'gse-app --help' for the option list."
                    ))
                }
            }
        }
        Ok(opts)
    }
}

fn main() {
    let opts = match Options::parse(std::env::args().skip(1)) {
        Ok(opts) => opts,
        Err(e) => {
            eprintln!("gse-app: {e}");
            std::process::exit(2);
        }
    };
    if opts.help {
        print!("{USAGE}");
        return;
    }
    if opts.version {
        println!("gse-app {VERSION}");
        return;
    }
    if let Err(e) = run(opts) {
        eprintln!("gse-app: {e}");
        std::process::exit(1);
    }
}

fn run(opts: Options) -> Result<(), String> {
    signals::install();
    let config_path = opts
        .config
        .clone()
        .unwrap_or_else(paths::default_config_path);

    let (mut config, config_warning) = Config::load(&config_path);
    if let Some(port) = &opts.port {
        config.port = port.clone();
    }
    if let Some(baud) = opts.baud {
        config.baud = baud;
    }
    if let Some(uart_baud) = opts.uart_baud {
        config.uart_baud = uart_baud;
    }
    if let Some(radio) = opts.radio {
        config.radio = radio;
    }
    if let Some(host) = &opts.host {
        config.host = host.clone();
    }
    if let Some(port) = opts.http_port {
        config.http_port = port;
    }
    config.sanitize();
    if let Err(e) = config.save(&config_path) {
        eprintln!(
            "[gse] warning: cannot save config to {}: {e}",
            config_path.display()
        );
    }

    let started = clock::unix_ms();
    let session = match SessionLog::create(&paths::logs_dir(), started) {
        Ok(log) => Some(log),
        Err(e) => {
            eprintln!("[gse] warning: cannot create session log: {e}");
            None
        }
    };
    let log_path = session
        .as_ref()
        .map(|s| s.path().display().to_string())
        .unwrap_or_else(|| "in memory".to_string());

    // Simulation runs against a TCP socket exposed by `gse-sim`. Start it if it
    // sits next to this executable and is not already listening.
    let mut port_override = None;
    if opts.sim {
        port_override = Some(SIM_SPEC.to_string());
        spawn_simulator(&opts);
    } else if config.port != "auto" {
        port_override = Some(config.port.clone());
    }

    let (listener, addr) = http::bind(&config.host, config.http_port)
        .map_err(|e| format!("cannot start the console on {}: {e}", config.host))?;
    let url = http::url_for(addr, &config.host);

    let port_hint = if opts.sim {
        SIM_SPEC.to_string()
    } else if config.port == "auto" {
        "auto-detect".to_string()
    } else {
        config.port.clone()
    };

    let mut app = AppState::new(config.clone(), opts.sim, url.clone(), port_hint);
    app.log_path = Some(log_path.clone());
    let token = state::make_token();
    let shared = Shared::new(
        app,
        session,
        token.clone(),
        config_path.clone(),
        opts.verbose,
    );

    if let Some(warning) = config_warning {
        shared.log(Level::Warn, Source::App, warning, Some("config"));
    }
    shared.log(
        Level::Ok,
        Source::App,
        format!("GSE {VERSION} host starting on {url}"),
        Some("boot"),
    );
    if opts.sim {
        shared.log(
            Level::Info,
            Source::App,
            format!("Simulation mode - link target {SIM_SPEC}"),
            Some("sim"),
        );
    }

    let (tx, rx) = mpsc::channel();
    let engine_shared = shared.clone();
    let engine_spec = port_override.clone();
    let verbose = opts.verbose;
    let engine_thread = thread::Builder::new()
        .name("gse-engine".to_string())
        .spawn(move || {
            Engine::new(engine_shared, rx, engine_spec, verbose).run();
        })
        .map_err(|e| format!("cannot start the link engine: {e}"))?;

    let _http_thread = thread::Builder::new()
        .name("gse-http".to_string())
        .spawn({
            let shared = shared.clone();
            let tx = tx.clone();
            move || http::serve(listener, shared, tx)
        })
        .map_err(|e| format!("cannot start the console: {e}"))?;

    println!("GSE {VERSION}");
    println!("  console  {url}");
    println!(
        "  link     {} @ {} baud{}",
        port_override
            .clone()
            .unwrap_or_else(|| "auto-detect".to_string()),
        config.baud,
        if opts.sim { " (simulator)" } else { "" }
    );
    println!("  config   {}", config_path.display());
    println!("  log      {log_path}");
    println!("Press Ctrl+C to stop.");

    if should_open_browser(&opts) {
        if let Err(e) = open_browser(&url) {
            eprintln!("[gse] cannot open a browser ({e}); open {url} manually");
        }
    }

    // Stay alive until Ctrl+C / SIGTERM, then ask the engine to stop.
    while !signals::requested() {
        thread::sleep(Duration::from_millis(120));
    }
    shared.log(
        Level::Info,
        Source::App,
        "Shutting down".to_string(),
        Some("shutdown"),
    );
    let _ = tx.send(engine::Request::Shutdown);
    let _ = engine_thread.join();
    // Give the HTTP threads a moment to flush the last frame.
    thread::sleep(Duration::from_millis(150));
    shared.log(
        Level::Info,
        Source::App,
        "Ground station stopped".to_string(),
        Some("shutdown"),
    );
    Ok(())
}

fn should_open_browser(opts: &Options) -> bool {
    if opts.no_browser {
        return false;
    }
    if opts.force_browser {
        return true;
    }
    #[cfg(target_os = "linux")]
    {
        let has_display =
            std::env::var_os("DISPLAY").is_some() || std::env::var_os("WAYLAND_DISPLAY").is_some();
        if !has_display {
            return false;
        }
    }
    true
}

fn open_browser(url: &str) -> std::io::Result<()> {
    #[cfg(target_os = "windows")]
    {
        let mut child = Command::new("cmd").args(["/C", "start", "", url]).spawn()?;
        thread::spawn(move || {
            let _ = child.wait();
        });
        Ok(())
    }
    #[cfg(target_os = "macos")]
    {
        let mut child = Command::new("open").arg(url).spawn()?;
        thread::spawn(move || {
            let _ = child.wait();
        });
        Ok(())
    }
    #[cfg(all(unix, not(target_os = "macos")))]
    {
        let mut child = Command::new("xdg-open").arg(url).spawn()?;
        thread::spawn(move || {
            let _ = child.wait();
        });
        Ok(())
    }
    #[cfg(not(any(unix, windows)))]
    {
        let _ = url;
        Err(std::io::Error::new(
            std::io::ErrorKind::Unsupported,
            "no browser launcher on this platform",
        ))
    }
}

/// Start `gse-sim` from next to the executable when running in simulation mode.
fn spawn_simulator(opts: &Options) {
    if std::net::TcpStream::connect(SIM_ADDR).is_ok() {
        return;
    }
    let dir = paths::exe_dir();
    let name = if cfg!(windows) {
        "gse-sim.exe"
    } else {
        "gse-sim"
    };
    let candidate = dir.join(name);
    if !candidate.is_file() {
        if opts.verbose {
            eprintln!(
                "[gse] no simulator next to the executable ({}); start 'gse-sim' manually",
                candidate.display()
            );
        }
        return;
    }
    match Command::new(&candidate)
        .args(["--listen", SIM_ADDR])
        .spawn()
    {
        Ok(mut child) => {
            thread::spawn(move || {
                let _ = child.wait();
            });
            thread::sleep(Duration::from_millis(250));
        }
        Err(e) => eprintln!("[gse] cannot start {}: {e}", candidate.display()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Options {
        Options::parse(args.iter().map(|s| s.to_string())).unwrap()
    }

    #[test]
    fn parses_flags_and_values() {
        let opts = parse(&["--sim", "--baud", "9600", "--http-port=9000", "--verbose"]);
        assert!(opts.sim);
        assert!(opts.verbose);
        assert_eq!(opts.baud, Some(9_600));
        assert_eq!(opts.http_port, Some(9_000));
    }

    #[test]
    fn inline_values_work() {
        let opts = parse(&["--port=tcp:127.0.0.1:8787", "--config=/tmp/x.json"]);
        assert_eq!(opts.port.as_deref(), Some("tcp:127.0.0.1:8787"));
        assert_eq!(opts.config.unwrap().to_string_lossy(), "/tmp/x.json");
    }

    #[test]
    fn unknown_and_incomplete_options_are_rejected() {
        assert!(Options::parse(["--bogus".to_string()]).is_err());
        assert!(Options::parse(["--baud".to_string()]).is_err());
        assert!(Options::parse(["--baud".to_string(), "fast".to_string()]).is_err());
    }
}
