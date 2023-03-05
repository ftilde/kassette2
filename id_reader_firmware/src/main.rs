#![no_std]
#![no_main]

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
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

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Id([u8; 4]);

fn fmt_nibble(nibble: u8) -> u8 {
    if nibble < 10 {
        nibble + b'0'
    } else {
        nibble - 10 + b'A'
    }
}

fn fmt_byte_to_nibbles(v: u8) -> [u8; 2] {
    let n_high = v >> 4;
    let n_low = v & 0xf;
    [fmt_nibble(n_high), fmt_nibble(n_low)]
}

impl core::fmt::Display for Id {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in &self.0 {
            let nibbles = fmt_byte_to_nibbles(*b);
            f.write_str(unsafe { core::str::from_utf8_unchecked(&nibbles) })?;
            write!(f, " ")?;
        }
        Ok(())
    }
}

struct IdReader<SPI, NSS> {
    prev_state: Option<Id>,
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
            prev_state: None,
        }
    }
    fn poll_event(&mut self) -> Option<IdReaderEvent> {
        let prev_id = self.prev_state.take();
        // For some reason, the second `reqa` after a `select` does not work if the card stays on
        // the reader. We thus retry exactly once here.
        if let Ok(a) = self.device.reqa().or_else(|_| self.device.reqa()) {
            if let Ok(n_uid) = self.device.select(&a) {
                let n_id = unpack_uid(n_uid);
                self.prev_state = Some(n_id);
                return match prev_id {
                    Some(id) if id == n_id => None,
                    Some(_) | None => Some(IdReaderEvent::New(n_id)),
                };
            }
        }
        self.prev_state = None;
        match prev_id {
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
    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut out = SerialOut {
            serial: &mut serial,
        };

        use core::fmt::Write;
        let _ = write!(out, "\r");

        if let Some(e) = id_reader.poll_event() {
            match e {
                IdReaderEvent::New(id) => {
                    let _ = write!(out, "New card: {}\r\n", id);
                    led_pin.set_high().unwrap();
                }
                IdReaderEvent::Removed => {
                    let _ = write!(out, "Card removed\r\n");
                    led_pin.set_low().unwrap();
                }
            }
        }
        delay.delay_ms(10);
    }
}
