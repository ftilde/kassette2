use rp_pico::{
    hal::{self, Clock},
    pac,
};

use crate::sdcard::{self, SDCardFile};

use embedded_hal::digital::v2::OutputPin;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let core = unsafe { pac::CorePeripherals::steal() };
    let mut pac = unsafe { pac::Peripherals::steal() };

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut sd = sdcard::init_sd(
        pins.gpio2,
        pins.gpio3,
        pins.gpio0,
        pins.gpio1,
        pac.SPI0,
        &mut pac.RESETS,
        &clocks,
    );

    {
        let mut fs = sdcard::SDCardController::init(&mut sd);

        if let Ok(mut file) = SDCardFile::open(
            &mut fs,
            "error.log",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        ) {
            use core::fmt::Write;
            let _ = writeln!(file, "===============================");
            let _ = writeln!(file, "{}", info);
        };
    }

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        for _ in 0..3 {
            let _ = led_pin.set_high();
            delay.delay_ms(200);
            let _ = led_pin.set_low();
            delay.delay_ms(200);
        }
        delay.delay_ms(400);
        for _ in 0..3 {
            let _ = led_pin.set_high();
            delay.delay_ms(600);
            let _ = led_pin.set_low();
            delay.delay_ms(600);
        }
        for _ in 0..3 {
            let _ = led_pin.set_high();
            delay.delay_ms(200);
            let _ = led_pin.set_low();
            delay.delay_ms(200);
        }
        delay.delay_ms(1000);
    }
}
