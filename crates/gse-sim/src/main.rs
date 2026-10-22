//! Desktop simulator for the two NUCLEO-H753ZI boards.
//!
//! Runs the real `gse-fw-core` ground bridge and vehicle logic against mock
//! hardware, connects them through a simulated LoRa radio (latency, optional
//! bit errors and packet loss) and exposes the ground board's USB serial port
//! as a TCP socket. The host application connects with `--port tcp:host:port`
//! or `--sim`, which is exactly what it does for the real USB CDC port.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::process;
use std::thread;
use std::time::{Duration, Instant};

use gse_fw_core::ground::Ground;
use gse_fw_core::mock::{MockGround, MockVehicle};
use gse_fw_core::vehicle::Vehicle;
use gse_proto::radio::DxLr42Level;
use gse_proto::RADIO_BAUD;

const TICK_MS: u64 = 2;
const VERSION: &str = env!("CARGO_PKG_VERSION");

const USAGE: &str = "\
gse-sim - two-board simulator for the NUCLEO-H753ZI + DX-LR42-24T12D (SX1281) link

USAGE:
    gse-sim [OPTIONS]

OPTIONS:
    --listen <ADDR>        TCP address for the ground board's USB port
                           (default 127.0.0.1:8787)
    --level <0..8>         Simulate DX-LR42-24T12D AT+LEVEL air-rate latency
    --latency <MS>         One-way radio latency override (default 35)
    --drop-ppm <PPM>       Drop this many packets per million (default 0)
    --corrupt-ppm <PPM>    Corrupt this many packets per million (default 0)
    --noise                Shorthand for --drop-ppm 200 --corrupt-ppm 2000
    --fill-seconds <S>     Vehicle fill timer, informational only
    --quiet                Only print the listening line
    -h, --help             Show this help
    -V, --version          Show the version
";

#[derive(Clone, Copy, Debug)]
struct RadioConfig {
    latency_ms: u32,
    drop_ppm: u32,
    corrupt_ppm: u32,
}

impl Default for RadioConfig {
    fn default() -> Self {
        RadioConfig {
            latency_ms: 35,
            drop_ppm: 0,
            corrupt_ppm: 0,
        }
    }
}

#[derive(Debug)]
struct Options {
    listen: String,
    radio: RadioConfig,
    quiet: bool,
    help: bool,
    version: bool,
}

impl Default for Options {
    fn default() -> Self {
        Options {
            listen: "127.0.0.1:8787".to_string(),
            radio: RadioConfig::default(),
            quiet: false,
            help: false,
            version: false,
        }
    }
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
                "--listen" => opts.listen = value("--listen")?,
                "--level" => {
                    let text = value("--level")?;
                    let code = text
                        .parse::<u8>()
                        .ok()
                        .and_then(DxLr42Level::from_code)
                        .ok_or_else(|| {
                            format!("invalid DX-LR42 level '{text}' (expected 0..=8)")
                        })?;
                    opts.radio.latency_ms = code.estimate_one_way_ms(25, RADIO_BAUD);
                }
                "--latency" => {
                    let text = value("--latency")?;
                    opts.radio.latency_ms = text
                        .parse::<u32>()
                        .map_err(|_| format!("invalid latency '{text}'"))?
                        .min(5_000);
                }
                "--drop-ppm" => {
                    let text = value("--drop-ppm")?;
                    opts.radio.drop_ppm = text
                        .parse::<u32>()
                        .map_err(|_| format!("invalid drop rate '{text}'"))?;
                }
                "--corrupt-ppm" => {
                    let text = value("--corrupt-ppm")?;
                    opts.radio.corrupt_ppm = text
                        .parse::<u32>()
                        .map_err(|_| format!("invalid corruption rate '{text}'"))?;
                }
                "--noise" => {
                    opts.radio.drop_ppm = 200;
                    opts.radio.corrupt_ppm = 2_000;
                }
                "--fill-seconds" => {
                    let text = value("--fill-seconds")?;
                    let _ = text
                        .parse::<u16>()
                        .map_err(|_| format!("invalid fill time '{text}'"))?;
                }
                "--quiet" | "-q" => opts.quiet = true,
                "--help" | "-h" => opts.help = true,
                "--version" | "-V" => opts.version = true,
                other => {
                    return Err(format!(
                        "unknown option '{other}'\nRun 'gse-sim --help' for the option list."
                    ))
                }
            }
        }
        Ok(opts)
    }
}

/// Small deterministic RNG (xorshift32); avoids a dependency on `rand`.
struct Rng(u32);

impl Rng {
    fn new(seed: u32) -> Self {
        Rng(if seed == 0 { 0x9E37_79B9 } else { seed })
    }

    fn next_u32(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    fn chance_ppm(&mut self, ppm: u32) -> bool {
        ppm > 0 && (self.next_u32() % 1_000_000) < ppm
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dest {
    ToVehicle,
    ToGround,
}

struct Packet {
    due: Instant,
    dest: Dest,
    bytes: Vec<u8>,
}

/// Simulated radio medium with latency and optional bit errors.
struct Radio {
    config: RadioConfig,
    queue: VecDeque<Packet>,
    rng: Rng,
    delivered: u64,
    dropped: u64,
    corrupted: u64,
}

impl Radio {
    fn new(config: RadioConfig, seed: u32) -> Self {
        Radio {
            config,
            queue: VecDeque::new(),
            rng: Rng::new(seed),
            delivered: 0,
            dropped: 0,
            corrupted: 0,
        }
    }

    fn transmit(&mut self, dest: Dest, bytes: Vec<u8>, now: Instant) {
        if bytes.is_empty() {
            return;
        }
        if self.rng.chance_ppm(self.config.drop_ppm) {
            self.dropped += 1;
            return;
        }
        let mut bytes = bytes;
        if self.rng.chance_ppm(self.config.corrupt_ppm) {
            let index = (self.rng.next_u32() as usize) % bytes.len();
            let bit = 1u8 << (self.rng.next_u32() % 8);
            bytes[index] ^= bit;
            self.corrupted += 1;
        }
        self.queue.push_back(Packet {
            due: now + Duration::from_millis(self.config.latency_ms as u64),
            dest,
            bytes,
        });
    }

    fn deliver(&mut self, now: Instant, ground: &mut MockGround, vehicle: &mut MockVehicle) {
        // The queue is ordered by insertion; latency is constant so it is also
        // ordered by due time.
        while let Some(packet) = self.queue.front() {
            if packet.due > now {
                break;
            }
            let packet = self.queue.pop_front().expect("front checked");
            match packet.dest {
                Dest::ToVehicle => vehicle.feed_bytes(&packet.bytes),
                Dest::ToGround => ground.radio_in.extend(packet.bytes.iter().copied()),
            }
            self.delivered += 1;
        }
    }
}

/// The two boards and the radio between them. These outlive any single host
/// connection: unplugging the laptop's USB cable does not reboot the flight
/// hardware, so the simulator must not forget the vehicle state either.
struct Boards {
    ground: Ground<MockGround>,
    vehicle: Vehicle<MockVehicle>,
    radio: Radio,
}

impl Boards {
    fn new(config: RadioConfig, seed: u32) -> Self {
        let mut mock_ground = MockGround::default();
        let mut mock_vehicle = MockVehicle::default();
        // Both local DX-LR42-24T12D modules emit `"Power On\r\n"` on power-up.
        mock_ground.radio_in.extend(b"Power On\r\n".iter().copied());
        mock_vehicle
            .radio_in
            .extend(b"Power On\r\n".iter().copied());
        Boards {
            ground: Ground::new(mock_ground),
            vehicle: Vehicle::new(mock_vehicle),
            radio: Radio::new(config, seed),
        }
    }

    /// Let both boards run for `ms` without a host attached, so link-loss
    /// failsafes and timers behave as they do on real hardware.
    fn advance(&mut self, ms: u32) {
        self.ground.hal_mut().advance(ms);
        self.vehicle.hal_mut().advance(ms);
    }
}

/// One host connection attached to the persistent [`Boards`].
struct Sim<'a> {
    boards: &'a mut Boards,
    stream: &'a mut TcpStream,
    pending_out: Vec<u8>,
    bytes_from_host: u64,
    bytes_to_host: u64,
}

impl<'a> Sim<'a> {
    fn new(boards: &'a mut Boards, stream: &'a mut TcpStream) -> Self {
        Sim {
            boards,
            stream,
            pending_out: Vec::new(),
            bytes_from_host: 0,
            bytes_to_host: 0,
        }
    }

    fn step(&mut self, elapsed_ms: u32) -> io::Result<()> {
        let ground_hal = self.boards.ground.hal_mut();
        ground_hal.advance(elapsed_ms);
        self.boards.vehicle.hal_mut().advance(elapsed_ms);

        // USB serial: host -> ground board.
        let mut buf = [0u8; 512];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "host closed the connection",
                    ))
                }
                Ok(n) => {
                    self.bytes_from_host = self.bytes_from_host.wrapping_add(n as u64);
                    self.boards.ground.hal_mut().feed_bytes_to_usb(&buf[..n]);
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }

        // Ground bridge: host frames -> radio.
        self.boards.ground.poll();
        let radio_out = std::mem::take(&mut self.boards.ground.hal_mut().radio_out);
        let now = Instant::now();
        self.boards.radio.transmit(Dest::ToVehicle, radio_out, now);
        let ground_hal = self.boards.ground.hal_mut();
        self.boards
            .radio
            .deliver(now, ground_hal, self.boards.vehicle.hal_mut());

        // Vehicle logic: commands in, acks/state out.
        self.boards.vehicle.poll();
        let radio_out = std::mem::take(&mut self.boards.vehicle.hal_mut().radio_out);
        let now = Instant::now();
        self.boards.radio.transmit(Dest::ToGround, radio_out, now);
        let ground_hal = self.boards.ground.hal_mut();
        self.boards
            .radio
            .deliver(now, ground_hal, self.boards.vehicle.hal_mut());

        // Ground bridge again so the radio -> USB path sees what just arrived.
        self.boards.ground.poll();
        let usb_out = std::mem::take(&mut self.boards.ground.hal_mut().usb_out);
        if !usb_out.is_empty() {
            self.bytes_to_host = self.bytes_to_host.wrapping_add(usb_out.len() as u64);
            self.pending_out.extend_from_slice(&usb_out);
        }
        self.flush_out()?;

        Ok(())
    }
}

impl Sim<'_> {
    /// Write as much buffered host-bound data as the socket accepts.
    fn flush_out(&mut self) -> io::Result<()> {
        while !self.pending_out.is_empty() {
            match self.stream.write(&self.pending_out) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::WriteZero,
                        "host socket closed",
                    ))
                }
                Ok(n) => {
                    self.pending_out.drain(..n);
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
        Ok(())
    }
}

fn main() {
    let opts = match Options::parse(std::env::args().skip(1)) {
        Ok(opts) => opts,
        Err(e) => {
            eprintln!("gse-sim: {e}");
            process::exit(2);
        }
    };
    if opts.help {
        print!("{USAGE}");
        return;
    }
    if opts.version {
        println!("gse-sim {VERSION}");
        return;
    }
    if let Err(e) = run(opts) {
        eprintln!("gse-sim: {e}");
        process::exit(1);
    }
}

fn run(opts: Options) -> Result<(), String> {
    let addr: SocketAddr = opts
        .listen
        .to_socket_addrs()
        .map_err(|e| format!("cannot resolve {}: {e}", opts.listen))?
        .next()
        .ok_or_else(|| format!("cannot resolve {}", opts.listen))?;
    let listener = TcpListener::bind(addr).map_err(|e| format!("cannot listen on {addr}: {e}"))?;
    let local = listener.local_addr().unwrap_or(addr);
    println!("gse-sim {VERSION} listening on {local}");
    if !opts.quiet {
        println!(
            "  radio latency {} ms, drop {} ppm, corruption {} ppm",
            opts.radio.latency_ms, opts.radio.drop_ppm, opts.radio.corrupt_ppm
        );
        println!("  connect with: gse-app --sim   (or --port tcp:{local})");
    }

    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(1);
    let mut boards = Boards::new(opts.radio, seed);
    let mut last_active = Instant::now();
    let mut client_id = 0u32;
    for incoming in listener.incoming() {
        match incoming {
            Ok(stream) => {
                client_id = client_id.wrapping_add(1);
                let id = client_id;
                let idle_ms = last_active.elapsed().as_millis().min(60_000) as u32;
                if idle_ms > 0 {
                    boards.advance(idle_ms);
                }
                if !opts.quiet {
                    println!("[sim] host #{id} connected from {:?}", stream.peer_addr());
                }
                match serve_client(&mut boards, stream, id) {
                    Ok((from_host, to_host)) => {
                        if !opts.quiet {
                            println!(
                                "[sim] host #{id} disconnected ({} bytes up, {} bytes down)",
                                from_host, to_host
                            );
                        }
                    }
                    Err(e) => {
                        if !opts.quiet {
                            println!("[sim] host #{id} ended: {e}");
                        }
                    }
                }
                last_active = Instant::now();
            }
            Err(e) => {
                eprintln!("[sim] accept error: {e}");
                thread::sleep(Duration::from_millis(100));
            }
        }
    }
    Ok(())
}

/// Convert real elapsed time into whole simulated milliseconds, carrying the
/// sub-millisecond remainder into the next call.
///
/// The mock boards run on an integer millisecond clock. Truncating every loop
/// iteration (and re-basing on the start of each step) threw the remainder
/// away, so the simulated clock ran several percent slow: a 60 s fill reached
/// the host's deadline with only ~56 s elapsed on the board, and the vehicle
/// rightly rejected the FILL OFF as an aborted sequence.
fn take_millis(elapsed: Duration, carry: &mut Duration) -> u32 {
    let total = elapsed + *carry;
    let ms = total.as_millis() as u32;
    *carry = total - Duration::from_millis(ms as u64);
    ms
}

fn serve_client(boards: &mut Boards, mut stream: TcpStream, _id: u32) -> io::Result<(u64, u64)> {
    stream.set_nodelay(true)?;
    stream.set_nonblocking(true)?;
    let mut sim = Sim::new(boards, &mut stream);
    let mut last = Instant::now();
    let mut carry = Duration::ZERO;
    loop {
        let step_start = Instant::now();
        let advance = take_millis(step_start.duration_since(last), &mut carry);
        last = step_start;
        match sim.step(advance) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => {
                return Ok((sim.bytes_from_host, sim.bytes_to_host))
            }
            Err(e) if e.kind() == io::ErrorKind::ConnectionReset => {
                return Ok((sim.bytes_from_host, sim.bytes_to_host))
            }
            Err(e) => return Err(e),
        }
        let spent = step_start.elapsed();
        if spent < Duration::from_millis(TICK_MS) {
            thread::sleep(Duration::from_millis(TICK_MS) - spent);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Options {
        Options::parse(args.iter().map(|s| s.to_string())).unwrap()
    }

    #[test]
    fn defaults_are_sane() {
        let opts = parse(&[]);
        assert_eq!(opts.listen, "127.0.0.1:8787");
        assert_eq!(opts.radio.latency_ms, 35);
        assert_eq!(opts.radio.drop_ppm, 0);
    }

    #[test]
    fn parses_radio_options() {
        let opts = parse(&[
            "--listen",
            "0.0.0.0:9000",
            "--latency=80",
            "--drop-ppm",
            "1000",
            "--corrupt-ppm=500",
        ]);
        assert_eq!(opts.listen, "0.0.0.0:9000");
        assert_eq!(opts.radio.latency_ms, 80);
        assert_eq!(opts.radio.drop_ppm, 1_000);
        assert_eq!(opts.radio.corrupt_ppm, 500);
    }

    #[test]
    fn bad_options_rejected() {
        assert!(Options::parse(["--nope".to_string()]).is_err());
        assert!(Options::parse(["--latency".to_string(), "soon".to_string()]).is_err());
        assert!(Options::parse(["--level".to_string(), "9".to_string()]).is_err());
    }

    #[test]
    fn level_option_derives_dx_lr42_airtime() {
        let fast = parse(&["--level", "3"]);
        let slow = parse(&["--level=1"]);
        assert!(slow.radio.latency_ms > fast.radio.latency_ms);
    }

    #[test]
    fn simulated_clock_does_not_drift() {
        // A loop that takes 2.4 ms per iteration must advance 2400 ms over
        // 1000 iterations. Truncating each one (the old behaviour) gave 2000 ms.
        let mut carry = Duration::ZERO;
        let mut total = 0u32;
        for _ in 0..1_000 {
            total += take_millis(Duration::from_micros(2_400), &mut carry);
        }
        assert_eq!(total, 2_400);

        // Sub-millisecond loops still make progress instead of starving the
        // clock forever.
        let mut carry = Duration::ZERO;
        let mut total = 0u32;
        for _ in 0..1_000 {
            total += take_millis(Duration::from_micros(300), &mut carry);
        }
        assert_eq!(total, 300);
        assert!(carry < Duration::from_millis(1));
    }

    #[test]
    fn rng_is_deterministic_and_bounded() {
        let mut a = Rng::new(42);
        let mut b = Rng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u32(), b.next_u32());
        }
        let mut rng = Rng::new(7);
        let mut hits = 0;
        for _ in 0..10_000 {
            if rng.chance_ppm(100_000) {
                hits += 1;
            }
        }
        assert!(hits > 700 && hits < 1_300, "hits={hits}");
        let mut rng = Rng::new(7);
        assert!(!rng.chance_ppm(0), "zero ppm must never fire");
    }
}
