//! ESP-IDF board layer shared by the ground and vehicle images.

// Each binary includes this file and uses only its own role's half.
#![allow(dead_code)]

use core::sync::atomic::{AtomicU32, Ordering};
use std::sync::Mutex;

use esp_idf_hal::delay::{FreeRtos, NON_BLOCK};
use esp_idf_hal::gpio::{AnyIOPin, AnyOutputPin, Output, OutputPin, PinDriver};
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, Resolution};
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::{config::Config as UartConfig, UartDriver};
use esp_idf_svc::espnow::{EspNow, PeerInfo, ReceiveInfo, SendStatus};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys;
use esp_idf_svc::wifi::{ClientConfiguration, Configuration, WifiDriver};
use gse_espnow::{
    EspNowGroundHal, EspNowIo, EspNowVehicleHal, PacketQueue, UsbIo, VehicleOutputs, ESPNOW_MTU,
};
use gse_fw_core::hal::Leds;

// GROUND_MAC, VEHICLE_MAC, ESPNOW_PMK, ESPNOW_LMK, ESPNOW_CHANNEL from build.rs.
include!(concat!(env!("OUT_DIR"), "/link_config.rs"));

/// Datagrams allowed in ESP-NOW's TX queue before `radio_ready` reports false.
const MAX_TX_IN_FLIGHT: u32 = 4;
/// Servo pulse widths at 50 Hz.
const SERVO_SAFE_US: u32 = 1_000;
const SERVO_ACTIVE_US: u32 = 2_000;
const SERVO_PERIOD_US: u32 = 20_000;
/// Ground host UART TX ring buffer; `pending` reports how much of it is in use.
const GROUND_UART_TX_BUFFER: usize = 4096;

static RX_QUEUE: Mutex<PacketQueue> = Mutex::new(PacketQueue::new());
static TX_IN_FLIGHT: AtomicU32 = AtomicU32::new(0);
static TX_FAILURES: AtomicU32 = AtomicU32::new(0);

/// Subscribe the calling task to the task watchdog, then poll forever.
///
/// Each pass yields for one FreeRTOS tick (1 ms, see `sdkconfig.defaults`) so
/// the idle tasks run and the idle-task watchdog stays quiet.
pub fn run_forever(mut poll: impl FnMut()) -> ! {
    esp_idf_svc::sys::esp!(unsafe { sys::esp_task_wdt_add(core::ptr::null_mut()) })
        .expect("esp_task_wdt_add");
    loop {
        poll();
        FreeRtos::delay_ms(1);
    }
}

/// Wi-Fi in station mode on a fixed channel, with one encrypted ESP-NOW peer.
pub struct NowIo {
    _wifi: WifiDriver<'static>,
    espnow: EspNow<'static>,
    peer: [u8; 6],
    leds: [PinDriver<'static, AnyOutputPin, Output>; 3],
    id: u32,
}

impl NowIo {
    pub fn new(
        modem: Modem,
        own_mac: [u8; 6],
        peer: [u8; 6],
        leds: [PinDriver<'static, AnyOutputPin, Output>; 3],
    ) -> Self {
        let sysloop = EspSystemEventLoop::take().expect("system event loop");
        let nvs = EspDefaultNvsPartition::take().expect("NVS partition");
        let mut wifi = WifiDriver::new(modem, sysloop, Some(nvs)).expect("Wi-Fi init");
        wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))
            .expect("Wi-Fi station mode");
        sys::esp!(unsafe {
            sys::esp_wifi_set_mac(sys::wifi_interface_t_WIFI_IF_STA, own_mac.as_ptr())
        })
        .expect("esp_wifi_set_mac");
        wifi.start().expect("Wi-Fi start");
        unsafe {
            sys::esp!(sys::esp_wifi_set_ps(sys::wifi_ps_type_t_WIFI_PS_NONE))
                .expect("esp_wifi_set_ps");
            sys::esp!(sys::esp_wifi_set_channel(
                ESPNOW_CHANNEL,
                sys::wifi_second_chan_t_WIFI_SECOND_CHAN_NONE,
            ))
            .expect("esp_wifi_set_channel");
        }

        let espnow = EspNow::take().expect("ESP-NOW init");
        espnow.set_pmk(&ESPNOW_PMK).expect("ESP-NOW PMK");
        espnow
            .add_peer(PeerInfo {
                peer_addr: peer,
                lmk: ESPNOW_LMK,
                channel: ESPNOW_CHANNEL,
                ifidx: sys::wifi_interface_t_WIFI_IF_STA,
                encrypt: true,
                ..Default::default()
            })
            .expect("ESP-NOW add peer");
        espnow
            .register_recv_cb(move |info: &ReceiveInfo, data: &[u8]| {
                // Only the configured peer is accepted; everything else on the
                // channel is ignored before it reaches the protocol decoder.
                if *info.src_addr != peer || data.len() > ESPNOW_MTU {
                    return;
                }
                if let Ok(mut queue) = RX_QUEUE.lock() {
                    let _ = queue.push(data);
                }
            })
            .expect("ESP-NOW receive callback");
        espnow
            .register_send_cb(|_mac: &[u8], status: SendStatus| {
                let _ = TX_IN_FLIGHT
                    .fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| n.checked_sub(1));
                if matches!(status, SendStatus::FAIL) {
                    TX_FAILURES.fetch_add(1, Ordering::Relaxed);
                }
            })
            .expect("ESP-NOW send callback");

        Self {
            _wifi: wifi,
            espnow,
            peer,
            leds,
            id: factory_id(),
        }
    }
}

/// Low four bytes of the factory (eFuse) MAC; unique per chip, unlike the
/// configured station MAC, which is the same for every board of one role.
fn factory_id() -> u32 {
    let mut mac = [0u8; 6];
    unsafe {
        sys::esp_efuse_mac_get_default(mac.as_mut_ptr());
    }
    u32::from_be_bytes([mac[2], mac[3], mac[4], mac[5]])
}

impl EspNowIo for NowIo {
    fn now_ms(&self) -> u32 {
        (unsafe { sys::esp_timer_get_time() } / 1000) as u32
    }

    fn receive(&mut self, out: &mut [u8]) -> usize {
        RX_QUEUE.lock().map(|mut q| q.pop(out)).unwrap_or(0)
    }

    fn send(&mut self, data: &[u8]) -> bool {
        TX_IN_FLIGHT.fetch_add(1, Ordering::AcqRel);
        if self.espnow.send(self.peer, data).is_ok() {
            true
        } else {
            TX_IN_FLIGHT.fetch_sub(1, Ordering::AcqRel);
            false
        }
    }

    fn tx_busy(&self) -> bool {
        TX_IN_FLIGHT.load(Ordering::Acquire) != 0
    }

    fn tx_ready(&self) -> bool {
        TX_IN_FLIGHT.load(Ordering::Acquire) < MAX_TX_IN_FLIGHT
    }

    fn set_leds(&mut self, leds: Leds) {
        set_output(&mut self.leds[0], leds.link);
        set_output(&mut self.leds[1], leds.activity);
        set_output(&mut self.leds[2], leds.fault);
    }

    fn kick_watchdog(&mut self) {
        unsafe {
            sys::esp_task_wdt_reset();
        }
    }

    fn device_id(&self) -> u32 {
        self.id
    }
}

pub struct VehicleActuators {
    fill: LedcDriver<'static>,
    srv: LedcDriver<'static>,
    ign: PinDriver<'static, AnyOutputPin, Output>,
}

impl VehicleActuators {
    fn set_servo(servo: &mut LedcDriver<'static>, on: bool) {
        let pulse_us = if on { SERVO_ACTIVE_US } else { SERVO_SAFE_US };
        let _ = servo.set_duty(servo.get_max_duty() * pulse_us / SERVO_PERIOD_US);
    }
}

impl VehicleOutputs for VehicleActuators {
    fn set_fill(&mut self, on: bool) {
        Self::set_servo(&mut self.fill, on);
    }

    fn set_srv(&mut self, on: bool) {
        Self::set_servo(&mut self.srv, on);
    }

    fn set_ign(&mut self, on: bool) {
        set_output(&mut self.ign, on);
    }
}

fn set_output(pin: &mut PinDriver<'static, AnyOutputPin, Output>, on: bool) {
    let _ = if on { pin.set_high() } else { pin.set_low() };
}

fn output(pin: impl OutputPin) -> PinDriver<'static, AnyOutputPin, Output> {
    let mut driver = PinDriver::output(pin.downgrade_output()).expect("GPIO output");
    let _ = driver.set_low();
    driver
}

pub struct GroundUsb {
    uart: UartDriver<'static>,
}

impl UsbIo for GroundUsb {
    fn read(&mut self, out: &mut [u8]) -> usize {
        self.uart.read(out, NON_BLOCK).unwrap_or(0)
    }

    fn write(&mut self, data: &[u8]) {
        let mut rest = data;
        while !rest.is_empty() {
            match self.uart.write(rest) {
                Ok(n) if n > 0 => rest = &rest[n..],
                _ => break,
            }
        }
    }

    fn pending(&self) -> usize {
        let free = self.uart.remaining_write().unwrap_or(0);
        GROUND_UART_TX_BUFFER.saturating_sub(free)
    }
}

pub type VehicleHal = EspNowVehicleHal<NowIo, VehicleActuators>;
pub type GroundHal = EspNowGroundHal<NowIo, GroundUsb>;

pub fn vehicle_hal(p: Peripherals) -> VehicleHal {
    // Outputs first: IGN low and both servos at their safe pulse before the
    // radio (which takes hundreds of milliseconds to start) is brought up.
    let ign = output(p.pins.gpio6);
    let timer = LedcTimerDriver::new(
        p.ledc.timer0,
        &TimerConfig::new()
            .frequency(50.Hz())
            .resolution(Resolution::Bits14),
    )
    .expect("LEDC timer");
    let timer: &'static LedcTimerDriver<'static, _> = Box::leak(Box::new(timer));
    let mut fill = LedcDriver::new(p.ledc.channel0, timer, p.pins.gpio7).expect("FILL PWM");
    VehicleActuators::set_servo(&mut fill, false);
    let mut srv = LedcDriver::new(p.ledc.channel1, timer, p.pins.gpio8).expect("SRV PWM");
    VehicleActuators::set_servo(&mut srv, false);
    let outputs = VehicleActuators { fill, srv, ign };

    let leds = [
        output(p.pins.gpio1),
        output(p.pins.gpio2),
        output(p.pins.gpio3),
    ];
    EspNowVehicleHal::new(NowIo::new(p.modem, VEHICLE_MAC, GROUND_MAC, leds), outputs)
}

pub fn ground_hal(p: Peripherals) -> GroundHal {
    let uart = UartDriver::new(
        p.uart0,
        p.pins.gpio43,
        p.pins.gpio44,
        Option::<AnyIOPin>::None,
        Option::<AnyIOPin>::None,
        &UartConfig::new()
            .baudrate(115_200.Hz())
            .rx_fifo_size(1024)
            .tx_fifo_size(GROUND_UART_TX_BUFFER),
    )
    .expect("host UART");
    let leds = [
        output(p.pins.gpio1),
        output(p.pins.gpio2),
        output(p.pins.gpio3),
    ];
    EspNowGroundHal::new(
        NowIo::new(p.modem, GROUND_MAC, VEHICLE_MAC, leds),
        GroundUsb { uart },
    )
}
