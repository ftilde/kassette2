#![no_std]
#![no_main]

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

use fugit::RateExtU32;

fn toggle(s: PinState) -> PinState {
    match s {
        PinState::Low => PinState::High,
        PinState::High => PinState::Low,
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

    let mut mfrc = mfrc522::Mfrc522::with_nss(spi, spi_csn).unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut led_pin = pins.led.into_push_pull_output();
    let mut state = PinState::Low;
    led_pin.set_state(state).unwrap();
    loop {
        if let Ok(a) = mfrc.reqa() {
            if let Ok(_uid) = mfrc.select(&a) {
                led_pin.set_state(state).unwrap();
                state = toggle(state);
            }
        }
        delay.delay_ms(500);
    }
}
