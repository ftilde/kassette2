#![no_std]
#![no_main]

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::PinState;
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use rp_pico::hal::Spi;

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::*;

use fugit::RateExtU32;

fn toggle(s: PinState) -> PinState {
    match s {
        PinState::Low => PinState::High,
        PinState::High => PinState::Low,
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Id([u8; 4]);

struct IdReader<SPI, NSS> {
    last_state: Option<Id>,
    device: mfrc522::Mfrc522<SPI, NSS>,
}

enum IdReaderEvent {
    New(Id),
    Removed,
}

fn unpack_uid(v: mfrc522::Uid) -> Id {
    let v: [u8; 4] = v.as_bytes().try_into().unwrap();
    Id(v)
}

impl<E, SPI, NSS> IdReader<SPI, NSS>
where
    E: core::fmt::Debug,
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    NSS: OutputPin,
{
    fn new(spi: SPI, nss: NSS) -> Self {
        let device = mfrc522::Mfrc522::with_nss(spi, nss).unwrap();
        Self {
            device,
            last_state: None,
        }
    }
    fn poll_event(&mut self, p: &mut dyn core::fmt::Write) -> Option<IdReaderEvent> {
        if let Ok(a) = self.device.reqa() {
            let _ = write!(p, "jou\r\n");
            if let Ok(n_uid) = self.device.select(&a) {
                let n_id = unpack_uid(n_uid);
                return match self.last_state {
                    Some(id) if id == n_id => None,
                    Some(_) | None => {
                        self.last_state = Some(n_id);
                        Some(IdReaderEvent::New(n_id))
                    }
                };
            }
        } else {
            let _ = write!(p, "nope\r\n");
        }
        self.last_state = None;
        match self.last_state {
            Some(_) => Some(IdReaderEvent::Removed),
            None => None,
        }
    }
}

struct SerialOut<'a, 'b, B>
where
    B: UsbBus,
{
    serial: &'a mut SerialPort<'b, B>,
}

impl<'a, 'b, B> core::fmt::Write for SerialOut<'a, 'b, B>
where
    B: UsbBus,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let _ = self.serial.write(s.as_bytes());
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //USB
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .product("Serial port")
        .device_class(USB_CLASS_CDC)
        .build();

    let spi: Spi<_, _, 8> = Spi::new(pac.SPI1).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        &embedded_hal::spi::MODE_0,
    );
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio8.into_mode::<hal::gpio::FunctionSpi>();
    let spi_csn = pins.gpio9.into_push_pull_output();

    let mut id_reader = IdReader::new(spi, spi_csn);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut led_pin = pins.led.into_push_pull_output();
    let mut pin_state = PinState::Low;
    led_pin.set_state(pin_state).unwrap();
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut out = SerialOut {
            serial: &mut serial,
        };

        if let Some(e) = id_reader.poll_event(&mut out) {
            match e {
                IdReaderEvent::New(_id) => led_pin.set_high().unwrap(),
                IdReaderEvent::Removed => led_pin.set_low().unwrap(),
            }
        }
    }
}
