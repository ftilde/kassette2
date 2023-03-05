#![no_std]
#![no_main]

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
use id_reader::*;

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
        // Somehow the USB connection seems to stall if we do not write anything for some time...
        let _ = write!(out, "\r");

        if let Some(e) = id_reader.poll_event() {
            match e {
                IdReaderEvent::New(id) => {
                    let _ = write!(out, "New card: \t{}\r\n", id);
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
