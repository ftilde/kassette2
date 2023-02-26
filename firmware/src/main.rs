#![no_std]
#![no_main]

extern crate alloc;

#[allow(unused)]
mod blink;
mod config;
mod output;
mod queue;
mod sdcard;

use acid_io::Read;
//use embedded_hal::digital::v2::OutputPin;
use embedded_sdmmc::Mode;
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

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

//#[alloc_error_handler]
//fn oom(_: Layout) -> ! {
//    loop {}
//}

fn data(sample_rate: u32, freq_hz: u32) -> impl Iterator<Item = u16> {
    let period_samples = sample_rate / freq_hz;
    let mut i: u32 = 0;
    const RANGE: u32 = 1 << crate::config::BITS_PER_SAMPLE;
    let sin_table: [u16; RANGE as usize] = core::array::from_fn(|i| {
        let f = i as f32 * core::f32::consts::TAU / (RANGE as f32);
        use micromath::F32Ext;
        let val = (((-f32::cos(f) + 1.0) * 0.5) * crate::config::MAX_SAMPLE as f32) as u32;
        val.min(crate::config::MAX_SAMPLE as u32) as u16
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

        val = (i * RANGE / period_samples) % RANGE;
        let val = sin_table[val as usize];

        i += 1;
        Some(val as u16)
    })
}

struct ReadableFile<'a, 'b, CS: hal::gpio::PinId> {
    fs: &'a mut sdcard::SDCardController<'b, CS>,
    file: &'a mut embedded_sdmmc::File,
}

impl<'a, 'b, CS: hal::gpio::PinId> Read for ReadableFile<'a, 'b, CS> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, acid_io::Error> {
        Ok(self.fs.read(&mut self.file, buf))
    }
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
    // First thing: Initialize the allocator
    {
        use core::mem::MaybeUninit;
        static mut HEAP_MEM: [MaybeUninit<u8>; config::HEAP_SIZE] =
            [MaybeUninit::uninit(); config::HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, config::HEAP_SIZE) }
    }

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

    let mut led_pin = pins.led.into_push_pull_output();

    output::setup_output(pac.PIO0, pac.TIMER, &mut pac.RESETS, pins.gpio15, cons);

    let mut sd = sdcard::init_sd(
        pins.gpio18,
        pins.gpio19,
        pins.gpio16,
        pins.gpio17,
        pac.SPI0,
        &mut pac.RESETS,
        &clocks,
    );
    let mut fs = sdcard::SDCardController::init(&mut sd);

    let mut f = fs.open("blk138.flc", Mode::ReadOnly);

    let mut data_fns = [
        data(40_783, 100),
        data(40_783, 200),
        data(40_783, 400),
        data(40_783, 800),
    ];
    let mut data_fn_i = 0;
    let mut data = 0;
    let mut i = 0;

    {
        let file = ReadableFile {
            fs: &mut fs,
            file: &mut f,
        };
        let Ok(mut file) = claxon::FlacReader::new(file) else {
            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_2_SHORT);
        };
        let mut blocks = file.blocks();

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
        let mut buf = alloc::vec::Vec::with_capacity(1024);
        let sin_test = false;
        loop {
            let Ok(frame) = blocks.read_next_or_eof(core::mem::take(&mut buf)) else {
                blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_3_SHORT);
            };
            let Some(frame) = frame else {
                break;
            };
            let channel = frame.channel(0); // We only have mono files here

            for v in channel {
                let sample = if sin_test {
                    let sample = data_fns[data_fn_i].next().unwrap();
                    i += 1;
                    if i == 40000 {
                        i = 0;
                        data_fn_i = (data_fn_i + 1) % data_fns.len();
                    }
                    sample
                } else {
                    ((v >> (16 - crate::config::BITS_PER_SAMPLE))
                        + crate::config::ZERO_SAMPLE as i32) as u16
                };
                while prod.push(sample).is_err() {}
            }

            buf = frame.into_buffer();

            //delay.delay_ms(20);
        }
    }
    fs.close(f);
    loop {
        while prod.push(data).is_ok() {
            data = data_fns[data_fn_i].next().unwrap();
            i += 1;
            if i == 40000 {
                i = 0;
                data_fn_i = (data_fn_i + 1) % data_fns.len();
            }
        }
        delay.delay_ms(20);
    }
}
