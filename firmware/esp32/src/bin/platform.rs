//! ESP-IDF board layer shared by the ground and vehicle images.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use esp_idf_hal::gpio::{
    Gpio1, Gpio2, Gpio3, Gpio43, Gpio44, Gpio6, Gpio7, Gpio8, Output, PinDriver,
};
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, Resolution, Speed};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::uart::{config::Config as UartConfig, UartDriver};
use esp_idf_sys as sys;
use gse_espnow::{
    EspNowGroundHal, EspNowIo, EspNowVehicleHal, PacketQueue, UsbIo, VehicleOutputs, ESPNOW_MTU,
};
use gse_fw_core::hal::Leds;

pub const GROUND_MAC: [u8; 6] = [0x24, 0x6F, 0x28, 0x00, 0x00, 0x01];
pub const VEHICLE_MAC: [u8; 6] = [0x24, 0x6F, 0x28, 0x00, 0x00, 0x02];

static RX_QUEUE: Mutex<RefCell<PacketQueue>> = Mutex::new(RefCell::new(PacketQueue::new()));
static LAST_SEND_MS: AtomicU32 = AtomicU32::new(0);

extern "C" fn on_receive(_info: *const sys::esp_now_recv_info_t, data: *const u8, len: i32) {
    if data.is_null() || len <= 0 || len as usize > ESPNOW_MTU {
        return;
    }
    let packet = unsafe { core::slice::from_raw_parts(data, len as usize) };
    critical_section::with(|cs| {
        let _ = RX_QUEUE.borrow(*cs).borrow_mut().push(packet);
    });
}

fn check(result: sys::esp_err_t, operation: &str) {
    if result != sys::ESP_OK {
        panic!("{operation} failed: ESP error {result}");
    }
}

pub fn init_esp_now(peer: [u8; 6]) {
    unsafe {
        check(sys::esp_netif_init(), "esp_netif_init");
        check(
            sys::esp_event_loop_create_default(),
            "esp_event_loop_create_default",
        );
        let config = sys::wifi_init_config_t::default();
        check(sys::esp_wifi_init(&config), "esp_wifi_init");
        check(
            sys::esp_wifi_set_mode(sys::wifi_mode_t_WIFI_MODE_STA),
            "esp_wifi_set_mode",
        );
        check(sys::esp_wifi_start(), "esp_wifi_start");
        check(sys::esp_now_init(), "esp_now_init");

        let mut info = sys::esp_now_peer_info_t::default();
        info.peer_addr.copy_from_slice(&peer);
        info.channel = 1;
        info.ifidx = sys::wifi_interface_t_WIFI_IF_STA;
        info.encrypt = false;
        check(sys::esp_now_add_peer(&info), "esp_now_add_peer");
        check(
            sys::esp_now_register_recv_cb(Some(on_receive)),
            "esp_now_register_recv_cb",
        );
    }
}

pub struct NowIo {
    peer: [u8; 6],
    leds: (
        PinDriver<'static, Gpio1, Output>,
        PinDriver<'static, Gpio2, Output>,
        PinDriver<'static, Gpio3, Output>,
    ),
    id: u32,
}

impl NowIo {
    pub fn new(
        peer: [u8; 6],
        leds: (
            PinDriver<'static, Gpio1, Output>,
            PinDriver<'static, Gpio2, Output>,
            PinDriver<'static, Gpio3, Output>,
        ),
    ) -> Self {
        init_esp_now(peer);
        Self { peer, leds, id: 0 }
    }
}

impl EspNowIo for NowIo {
    fn now_ms(&self) -> u32 {
        unsafe { (sys::esp_timer_get_time() as u64 / 1000) as u32 }
    }

    fn receive(&mut self, out: &mut [u8]) -> usize {
        critical_section::with(|cs| RX_QUEUE.borrow(*cs).borrow_mut().pop(out))
    }

    fn send(&mut self, data: &[u8]) -> bool {
        if data.len() > ESPNOW_MTU {
            return false;
        }
        let result = unsafe { sys::esp_now_send(self.peer.as_ptr(), data.as_ptr(), data.len()) };
        if result == sys::ESP_OK {
            LAST_SEND_MS.store(self.now_ms(), Ordering::Relaxed);
            true
        } else {
            false
        }
    }

    fn set_leds(&mut self, leds: Leds) {
        set_output(&mut self.leds.0, leds.link);
        set_output(&mut self.leds.1, leds.activity);
        set_output(&mut self.leds.2, leds.fault);
    }

    fn kick_watchdog(&mut self) {
        unsafe {
            let _ = sys::esp_task_wdt_reset();
        }
    }

    fn device_id(&self) -> u32 {
        self.id
    }
}

pub struct VehicleActuators {
    fill: LedcDriver<'static>,
    srv: LedcDriver<'static>,
    ign: PinDriver<'static, Gpio6, Output>,
}

impl VehicleActuators {
    pub fn new(peripherals: &Peripherals) -> Self {
        let timer_config = TimerConfig::new()
            .frequency(50.Hz())
            .resolution(Resolution::Bits12)
            .speed_mode(Speed::LowSpeed);
        let fill_timer = Box::leak(Box::new(
            LedcTimerDriver::new(peripherals.ledc.timer0, &timer_config).unwrap(),
        ));
        let srv_timer = Box::leak(Box::new(
            LedcTimerDriver::new(peripherals.ledc.timer1, &timer_config).unwrap(),
        ));
        let mut fill = LedcDriver::new(
            peripherals.ledc.channel0,
            fill_timer,
            peripherals.pins.gpio7,
        )
        .unwrap();
        let mut srv =
            LedcDriver::new(peripherals.ledc.channel1, srv_timer, peripherals.pins.gpio8).unwrap();
        let mut ign = PinDriver::output(peripherals.pins.gpio6).unwrap();
        let safe_duty = fill.max_duty() / 20;
        fill.set_duty(safe_duty).unwrap();
        srv.set_duty(safe_duty).unwrap();
        ign.set_low().unwrap();
        Self { fill, srv, ign }
    }
}

impl VehicleOutputs for VehicleActuators {
    fn set_fill(&mut self, on: bool) {
        let duty = if on {
            self.fill.max_duty() * 2 / 20
        } else {
            self.fill.max_duty() / 20
        };
        let _ = self.fill.set_duty(duty);
    }

    fn set_srv(&mut self, on: bool) {
        let duty = if on {
            self.srv.max_duty() * 2 / 20
        } else {
            self.srv.max_duty() / 20
        };
        let _ = self.srv.set_duty(duty);
    }

    fn set_ign(&mut self, on: bool) {
        set_output(&mut self.ign, on);
    }
}

fn set_output<P: esp_idf_hal::gpio::OutputPin>(pin: &mut PinDriver<'static, P, Output>, on: bool) {
    if on {
        let _ = pin.set_high();
    } else {
        let _ = pin.set_low();
    }
}

pub struct GroundUsb {
    uart: UartDriver<'static>,
}

impl GroundUsb {
    pub fn new(peripherals: &Peripherals) -> Self {
        let config = UartConfig::new().baudrate(115_200);
        let uart = UartDriver::new(
            peripherals.uart0,
            peripherals.pins.gpio43,
            peripherals.pins.gpio44,
            Option::<esp_idf_hal::gpio::AnyIOPin>::None,
            Option::<esp_idf_hal::gpio::AnyIOPin>::None,
            &config,
        )
        .unwrap();
        Self { uart }
    }
}

impl UsbIo for GroundUsb {
    fn read(&mut self, out: &mut [u8]) -> usize {
        self.uart.read(out, 0).unwrap_or(0)
    }

    fn write(&mut self, data: &[u8]) {
        let _ = self.uart.write(data);
    }
}

pub type VehicleHal = EspNowVehicleHal<NowIo, VehicleActuators>;
pub type GroundHal = EspNowGroundHal<NowIo, GroundUsb>;

pub fn vehicle_hal(peripherals: &Peripherals) -> VehicleHal {
    let leds = (
        PinDriver::output(peripherals.pins.gpio1).unwrap(),
        PinDriver::output(peripherals.pins.gpio2).unwrap(),
        PinDriver::output(peripherals.pins.gpio3).unwrap(),
    );
    EspNowVehicleHal::new(
        NowIo::new(GROUND_MAC, leds),
        VehicleActuators::new(peripherals),
    )
}

pub fn ground_hal(peripherals: &Peripherals) -> GroundHal {
    let leds = (
        PinDriver::output(peripherals.pins.gpio1).unwrap(),
        PinDriver::output(peripherals.pins.gpio2).unwrap(),
        PinDriver::output(peripherals.pins.gpio3).unwrap(),
    );
    EspNowGroundHal::new(NowIo::new(VEHICLE_MAC, leds), GroundUsb::new(peripherals))
}
