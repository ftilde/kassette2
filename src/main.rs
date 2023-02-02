#![no_std]
#![no_main]

mod blink;
mod config;
mod output;
mod queue;

use fugit::RateExtU32;
//use defmt::info;
//use defmt_rtt as _;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;

use embedded_sdmmc::{filesystem::Mode, Controller, SdMmcSpi, TimeSource, Timestamp, VolumeIdx};

/// A dummy timesource, which is mostly important for creating files.
#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

fn data(sample_rate: u32, freq_hz: u32) -> impl Iterator<Item = u8> {
    let period_samples = sample_rate / freq_hz;
    let mut i: u32 = 0;
    let sin_table: [u8; 256] = core::array::from_fn(|i| {
        let f = i as f32 * core::f32::consts::TAU / 256.0;
        use micromath::F32Ext;
        let val = (((-f32::cos(f) + 1.0) * 0.5) * u8::MAX as f32) as u32;
        val.min(255) as u8
    });
    core::iter::from_fn(move || {
        //let f = i as f32 * core::f32::consts::TAU / (period_samples as f32);
        //use micromath::F32Ext;
        //let val = (((-f32::cos(f) + 1.0) * 0.5) * u8::MAX as f32) as u32;

        let val;
        //if (i / period_samples) % 2 == 0 {
        //    val = 255;
        //} else {
        //    val = 0;
        //}

        val = (i * 256 / period_samples) % 256;
        let val = sin_table[val as usize];

        i += 1;
        Some(val as u8)
    })
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    let mut rb = queue::SampleQueue::default();
    let (mut prod, cons) = rb.split_ref();
    // Safety: Transmute to 'static lifetime. This is fine since main actually never returns.
    let cons = unsafe { core::mem::transmute(cons) };

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // ----------------------------------------------------------------------------
    // PIO stuff ------------------------------------------------------------------
    // ----------------------------------------------------------------------------

    output::setup_output(pac.PIO0, pac.TIMER, &mut pac.RESETS, pins.gpio15, cons);

    // ----------------------------------------------------------------------------
    // SD-Card stuff---------------------------------------------------------------
    // ----------------------------------------------------------------------------

    let mut led_pin = pins.led.into_push_pull_output();

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();
    let spi_cs = pins.gpio17.into_push_pull_output();

    // Create an SPI driver instance for the SPI0 device
    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut sdspi = SdMmcSpi::new(spi, spi_cs);

    // Next we need to aquire the block device and initialize the
    // communication with the SD card.
    let block = match sdspi.acquire() {
        Ok(block) => block,
        Err(_e) => {
            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_2_SHORT);
        }
    };

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut cont = Controller::new(block, DummyTimesource::default());

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    match cont.device().card_size_bytes() {
        Ok(_size) => {}
        Err(_e) => {
            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_3_SHORT);
        }
    }

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut volume = match cont.get_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(_e) => {
            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_4_SHORT);
        }
    };

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    // After we have the volume (partition) of the drive we got to open the
    // root directory:
    let dir = match cont.open_root_dir(&volume) {
        Ok(dir) => dir,
        Err(_e) => {
            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_5_SHORT);
        }
    };

    let mut file = match cont.open_file_in_dir(&mut volume, &dir, "bibi.bin", Mode::ReadOnly) {
        Ok(dir) => dir,
        Err(_e) => {
            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_6_SHORT);
        }
    };

    // ----------------------------------------------------------------------------
    // Main loop! -----------------------------------------------------------------
    // ----------------------------------------------------------------------------

    //let mut data_fns = [
    //    data(40_783, 100),
    //    data(40_783, 200),
    //    data(40_783, 400),
    //    data(40_783, 800),
    //];
    //let mut data_fn_i = 0;
    //let mut data = 0;
    //let mut i = 0;
    let mut buf = [0u8; 1024];
    loop {
        let read_count = cont.read(&volume, &mut file, &mut buf).unwrap();
        if read_count == 0 {
            break;
        }
        let mut begin = 0;
        let end = read_count;
        while begin < end {
            let pushed = prod.push_slice(&buf[begin..end]);
            begin += pushed;
        }

        //delay.delay_ms(20);
    }
    cont.close_file(&volume, file).unwrap();

    let mut data_fns = [
        data(40_783, 100),
        data(40_783, 200),
        data(40_783, 400),
        data(40_783, 800),
    ];
    let mut data_fn_i = 0;
    let mut data = 0;
    let mut i = 0;
    loop {
        while prod.push(data).is_ok() {
            data = data_fns[data_fn_i].next().unwrap().min(MAX_VALUE as u8);
            i += 1;
            if i == 40000 {
                i = 0;
                data_fn_i = (data_fn_i + 1) % data_fns.len();
            }
        }
        delay.delay_ms(20);
    }
}

const MAX_VALUE: u32 = u8::MAX as u32 - 1;
