//! Desktop simulator for the two NUCLEO-H753ZI boards.
//!
//! Runs the real `gse-fw-core` ground bridge and vehicle logic against mock
//! hardware, connects them through a simulated 2.4 GHz radio (`DX-LR42-24T12D`
//! UART LoRa or `E28-2G4M27S` SPI1 `500 mW` LoRa, enforcing active module
//! agreement and realistic UART/SPI airtime latency) and exposes the ground
//! board's USB serial port as a TCP socket.

use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream, ToSocketAddrs};
use std::process;
use std::thread;
use std::time::{Duration, Instant};

use gse_fw_core::ground::Ground;
use gse_fw_core::mock::{MockGround, MockVehicle};
use gse_fw_core::vehicle::Vehicle;
use gse_proto::radio::{e28, DxLr42Baud, DxLr42Level, RadioModule};

const TICK_MS: u64 = 2;
const VERSION: &str = env!("CARGO_PKG_VERSION");

const USAGE: &str = "\
gse-sim - two-board simulator for NUCLEO-H753ZI + DX-LR42-24T12D / E28-2G4M27S

USAGE:
    gse-sim [OPTIONS]

OPTIONS:
    --listen <ADDR>        TCP address for the ground board's USB port
                           (default 127.0.0.1:8787)
    --radio <MODULE>       Initial radio module: dx-lr42-24t12d (default) or e28-2g4m27s
    --uart-baud <BPS>      Initial MCU <-> UART radio baud rate (default 9600)
    --level <0..8>         Simulate DX-LR42-24T12D AT+LEVEL air-rate latency
    --latency <MS>         Fixed one-way radio latency override
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
    initial_radio: RadioModule,
    initial_uart_baud: DxLr42Baud,
    level: DxLr42Level,
    latency_override_ms: Option<u32>,
    drop_ppm: u32,
    corrupt_ppm: u32,
}

impl Default for RadioConfig {
    fn default() -> Self {
        RadioConfig {
            initial_radio: RadioModule::DxLr42,
            initial_uart_baud: DxLr42Baud::Baud9600,
            level: DxLr42Level::Level3,
            latency_override_ms: None,
            drop_ppm: 0,
            corrupt_ppm: 0,
        }
    }
}

impl RadioConfig {
    fn packet_latency_ms(&self, radio: RadioModule, uart_baud: DxLr42Baud, bytes: usize) -> u32 {
        if let Some(fixed) = self.latency_override_ms {
            return fixed;
        }
        match radio {
            RadioModule::DxLr42 => self
                .level
                .estimate_one_way_ms(bytes.max(16), uart_baud.bps()),
            RadioModule::E28_2G4M27S => e28::estimate_one_way_ms(bytes.max(16)),
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
                "--radio" => {
                    let text = value("--radio")?;
                    opts.radio.initial_radio = RadioModule::parse(&text).ok_or_else(|| {
                        format!(
                            "invalid radio module '{text}' (expected dx-lr42-24t12d or e28-2g4m27s)"
                        )
                    })?;
                }
                "--uart-baud" => {
                    let text = value("--uart-baud")?;
                    opts.radio.initial_uart_baud = text
                        .parse::<u32>()
                        .ok()
                        .and_then(DxLr42Baud::from_bps)
                        .ok_or_else(|| {
                            format!(
                                "invalid UART baud '{text}' (expected 2400, 4800, 9600, 19200, 38400, 57600, or 115200)"
                            )
                        })?;
                }
                "--level" => {
                    let text = value("--level")?;
                    opts.radio.level = text
                        .parse::<u8>()
                        .ok()
                        .and_then(DxLr42Level::from_code)
                        .ok_or_else(|| {
                            format!("invalid DX-LR42 level '{text}' (expected 0..=8)")
                        })?;
                }
                "--latency" => {
                    let text = value("--latency")?;
                    opts.radio.latency_override_ms = Some(
                        text.parse::<u32>()
                            .map_err(|_| format!("invalid latency '{text}'"))?
                            .min(5_000),
                    );
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
    radio: RadioModule,
    bytes: Vec<u8>,
}

/// Simulated 2.4 GHz RF medium enforcing transceiver compatibility, airtime latency,
/// and optional bit errors / packet loss.
struct Radio {
    config: RadioConfig,
    queue: Vec<Packet>,
    rng: Rng,
    delivered: u64,
    dropped: u64,
    corrupted: u64,
}

impl Radio {
    fn new(config: RadioConfig, seed: u32) -> Self {
        Radio {
            config,
            queue: Vec::new(),
            rng: Rng::new(seed),
            delivered: 0,
            dropped: 0,
            corrupted: 0,
        }
    }

    fn transmit(
        &mut self,
        dest: Dest,
        sender_radio: RadioModule,
        sender_uart_baud: DxLr42Baud,
        bytes: Vec<u8>,
        now: Instant,
    ) {
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
        let delay_ms = self
            .config
            .packet_latency_ms(sender_radio, sender_uart_baud, bytes.len());
        self.queue.push(Packet {
            due: now + Duration::from_millis(delay_ms as u64),
            dest,
            radio: sender_radio,
            bytes,
        });
    }

    fn deliver(&mut self, now: Instant, ground: &mut MockGround, vehicle: &mut MockVehicle) {
        let mut idx = 0usize;
        while idx < self.queue.len() {
            if self.queue[idx].due > now {
                idx += 1;
                continue;
            }
            let packet = self.queue.remove(idx);
            match packet.dest {
                Dest::ToVehicle => {
                    if packet.radio == vehicle.link.radio {
                        vehicle.feed_bytes(&packet.bytes);
                        self.delivered += 1;
                    } else {
                        self.dropped += 1;
                    }
                }
                Dest::ToGround => {
                    if packet.radio == ground.link.radio {
                        ground.radio_in.extend(packet.bytes.iter().copied());
                        self.delivered += 1;
                    } else {
                        self.dropped += 1;
                    }
                }
            }
        }
    }
}

/// The two boards and the radio between them.
struct Boards {
    ground: Ground<MockGround>,
    vehicle: Vehicle<MockVehicle>,
    radio: Radio,
}

impl Boards {
    fn new(config: RadioConfig, seed: u32) -> Self {
        let mut mock_ground = MockGround::default();
        let mut mock_vehicle = MockVehicle::default();
        mock_ground.link.radio = config.initial_radio;
        mock_ground.link.uart_baud = config.initial_uart_baud;
        mock_vehicle.link.radio = config.initial_radio;
        mock_vehicle.link.uart_baud = config.initial_uart_baud;
        if config.initial_radio == RadioModule::DxLr42 {
            mock_ground.radio_in.extend(b"Power On\r\n".iter().copied());
            mock_vehicle
                .radio_in
                .extend(b"Power On\r\n".iter().copied());
        }
        Boards {
            ground: Ground::new(mock_ground),
            vehicle: Vehicle::new(mock_vehicle),
            radio: Radio::new(config, seed),
        }
    }

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

        // Ground bridge: host frames -> radio (captured with Ground's active radio BEFORE next poll).
        self.boards.ground.poll();
        let g_link = self.boards.ground.hal().link;
        let radio_out = std::mem::take(&mut self.boards.ground.hal_mut().radio_out);
        let now = Instant::now();
        self.boards.radio.transmit(
            Dest::ToVehicle,
            g_link.radio,
            g_link.uart_baud,
            radio_out,
            now,
        );
        let ground_hal = self.boards.ground.hal_mut();
        self.boards
            .radio
            .deliver(now, ground_hal, self.boards.vehicle.hal_mut());

        // Vehicle logic: commands in, acks/state out (captured with Vehicle's active radio BEFORE switchover).
        self.boards.vehicle.poll();
        let v_link = self.boards.vehicle.hal().link;
        let radio_out = std::mem::take(&mut self.boards.vehicle.hal_mut().radio_out);
        let now = Instant::now();
        self.boards.radio.transmit(
            Dest::ToGround,
            v_link.radio,
            v_link.uart_baud,
            radio_out,
            now,
        );
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
            "  radio {}, uart {} baud, drop {} ppm, corruption {} ppm",
            opts.radio.initial_radio.part_number(),
            opts.radio.initial_uart_baud.bps(),
            opts.radio.drop_ppm,
            opts.radio.corrupt_ppm
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
    use gse_proto::{link_cfg_flags, Endpoint, LinkConfig, Message, Ping, UsbBaud};

    fn parse(args: &[&str]) -> Options {
        Options::parse(args.iter().map(|s| s.to_string())).unwrap()
    }

    #[test]
    fn defaults_are_sane() {
        let opts = parse(&[]);
        assert_eq!(opts.listen, "127.0.0.1:8787");
        assert_eq!(opts.radio.initial_radio, RadioModule::DxLr42);
        assert_eq!(opts.radio.initial_uart_baud, DxLr42Baud::Baud9600);
        assert_eq!(opts.radio.drop_ppm, 0);
    }

    #[test]
    fn parses_radio_and_baud_options() {
        let opts = parse(&[
            "--listen",
            "0.0.0.0:9000",
            "--radio=e28-2g4m27s",
            "--uart-baud",
            "115200",
            "--latency=80",
            "--drop-ppm",
            "1000",
            "--corrupt-ppm=500",
        ]);
        assert_eq!(opts.listen, "0.0.0.0:9000");
        assert_eq!(opts.radio.initial_radio, RadioModule::E28_2G4M27S);
        assert_eq!(opts.radio.initial_uart_baud, DxLr42Baud::Baud115200);
        assert_eq!(opts.radio.latency_override_ms, Some(80));
        assert_eq!(opts.radio.drop_ppm, 1_000);
        assert_eq!(opts.radio.corrupt_ppm, 500);
    }

    #[test]
    fn bad_options_rejected() {
        assert!(Options::parse(["--nope".to_string()]).is_err());
        assert!(Options::parse(["--latency".to_string(), "soon".to_string()]).is_err());
        assert!(Options::parse(["--level".to_string(), "9".to_string()]).is_err());
        assert!(Options::parse(["--radio".to_string(), "lora99".to_string()]).is_err());
        assert!(Options::parse(["--uart-baud".to_string(), "14400".to_string()]).is_err());
    }

    #[test]
    fn level_and_module_options_scale_airtime() {
        let fast = parse(&["--level", "3"]);
        let slow = parse(&["--level=1"]);
        let e28_opts = parse(&["--radio=e28-2g4m27s"]);
        assert!(
            slow.radio
                .packet_latency_ms(RadioModule::DxLr42, DxLr42Baud::Baud9600, 25)
                > fast
                    .radio
                    .packet_latency_ms(RadioModule::DxLr42, DxLr42Baud::Baud9600, 25)
        );
        assert!(
            e28_opts
                .radio
                .packet_latency_ms(RadioModule::E28_2G4M27S, DxLr42Baud::Baud9600, 25)
                < fast
                    .radio
                    .packet_latency_ms(RadioModule::DxLr42, DxLr42Baud::Baud9600, 25)
        );
    }

    #[test]
    fn two_phase_handover_switches_both_boards_to_e28_without_dropping_link() {
        let cfg = RadioConfig {
            latency_override_ms: Some(0),
            ..RadioConfig::default()
        };
        let mut boards = Boards::new(cfg, 99);
        let now = Instant::now();

        // Phase 1: send LinkConfig to Vehicle while both boards are on DX-LR42.
        let req = LinkConfig {
            req_id: 1,
            usb_baud: UsbBaud::Baud230400,
            uart_baud: DxLr42Baud::Baud57600,
            radio: RadioModule::E28_2G4M27S,
            flags: link_cfg_flags::PERSIST,
        };
        boards.ground.hal_mut().feed_usb_frame(
            &Message::LinkConfig(req),
            Endpoint::Host,
            Endpoint::Vehicle,
            1,
        );
        boards.ground.poll();
        let g_out = std::mem::take(&mut boards.ground.hal_mut().radio_out);
        boards.radio.transmit(
            Dest::ToVehicle,
            RadioModule::DxLr42,
            DxLr42Baud::Baud9600,
            g_out,
            now,
        );
        {
            let g_hal = boards.ground.hal_mut();
            boards.radio.deliver(now, g_hal, boards.vehicle.hal_mut());
        }

        // Vehicle emits LinkConfigAck on DX-LR42 BEFORE switching to E28!
        boards.vehicle.poll();
        assert_eq!(boards.vehicle.hal().link.radio, RadioModule::DxLr42);
        let v_out = std::mem::take(&mut boards.vehicle.hal_mut().radio_out);
        boards.radio.transmit(
            Dest::ToGround,
            RadioModule::DxLr42,
            DxLr42Baud::Baud9600,
            v_out,
            now,
        );
        {
            let g_hal = boards.ground.hal_mut();
            boards.radio.deliver(now, g_hal, boards.vehicle.hal_mut());
        }
        // Vehicle now finishes switching to E28, and Ground forwards Vehicle's ACK to USB!
        boards.vehicle.poll();
        assert_eq!(boards.vehicle.hal().link.radio, RadioModule::E28_2G4M27S);
        boards.ground.poll();
        let usb_frames = boards.ground.hal_mut().drain_usb_frames();
        assert!(usb_frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::LinkConfigAck(a)) if a.req_id == 1)));

        // Phase 2: send LinkConfig to Ground -> Ground ACKs over USB then switches to E28 + 230400!
        boards.ground.hal_mut().feed_usb_frame(
            &Message::LinkConfig(req),
            Endpoint::Host,
            Endpoint::Ground,
            2,
        );
        boards.ground.poll();
        let ack_frames = boards.ground.hal_mut().drain_usb_frames();
        assert!(ack_frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::LinkConfigAck(a)) if a.req_id == 1)));
        boards.ground.poll();
        assert_eq!(boards.ground.hal().link.radio, RadioModule::E28_2G4M27S);
        assert_eq!(boards.ground.hal().link.usb_baud, UsbBaud::Baud230400);

        // End-to-end PING now crosses over E28-2G4M27S cleanly!
        boards.ground.hal_mut().feed_usb_frame(
            &Message::Ping(Ping {
                nonce: 0x1234,
                host_ms: 50,
            }),
            Endpoint::Host,
            Endpoint::Vehicle,
            3,
        );
        boards.ground.poll();
        let g_out = std::mem::take(&mut boards.ground.hal_mut().radio_out);
        boards.radio.transmit(
            Dest::ToVehicle,
            boards.ground.hal().link.radio,
            boards.ground.hal().link.uart_baud,
            g_out,
            now,
        );
        {
            let g_hal = boards.ground.hal_mut();
            boards.radio.deliver(now, g_hal, boards.vehicle.hal_mut());
        }
        boards.vehicle.poll();
        let v_out = std::mem::take(&mut boards.vehicle.hal_mut().radio_out);
        boards.radio.transmit(
            Dest::ToGround,
            boards.vehicle.hal().link.radio,
            boards.vehicle.hal().link.uart_baud,
            v_out,
            now,
        );
        {
            let g_hal = boards.ground.hal_mut();
            boards.radio.deliver(now, g_hal, boards.vehicle.hal_mut());
        }
        boards.ground.poll();
        let pong_frames = boards.ground.hal_mut().drain_usb_frames();
        assert!(pong_frames
            .iter()
            .any(|f| matches!(f.message(), Some(Message::Pong(p)) if p.nonce == 0x1234)));
    }

    #[test]
    fn simulated_clock_does_not_drift() {
        let mut carry = Duration::ZERO;
        let mut total = 0u32;
        for _ in 0..1_000 {
            total += take_millis(Duration::from_micros(2_400), &mut carry);
        }
        assert_eq!(total, 2_400);

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
