#![no_std]
#![no_main]

extern crate alloc;

#[allow(unused)]
mod blink;
mod output;
mod queue;
mod sdcard;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use acid_io::Read;
use cortex_m::prelude::_embedded_hal_timer_CountDown;
use embedded_hal::digital::v2::OutputPin;
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
use rp_pico::hal::Spi;

use embedded_alloc::Heap;

use fugit::ExtU32;
use fugit::RateExtU32;

use id_reader::*;

#[global_allocator]
static HEAP: Heap = Heap::empty();

//#[alloc_error_handler]
//fn oom(_: Layout) -> ! {
//    loop {}
//}

fn data(freq_hz: u32) -> impl Iterator<Item = u16> {
    let period_samples = config::SAMPLE_RATE / freq_hz;
    let mut i: u32 = 0;
    const RANGE: u32 = 1 << config::BITS_PER_SAMPLE;
    let sin_table: [u16; RANGE as usize] = core::array::from_fn(|i| {
        let f = i as f32 * core::f32::consts::TAU / (RANGE as f32);
        use micromath::F32Ext;
        let val = (((-f32::cos(f) + 1.0) * 0.5) * config::MAX_SAMPLE as f32) as u32;
        val.min(config::MAX_SAMPLE as u32) as u16
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
    fs: &'a RefCell<sdcard::SDCardController<'b, CS>>,
    file: MaybeUninit<embedded_sdmmc::File>,
}

impl<'a, 'b, CS: hal::gpio::PinId> Read for ReadableFile<'a, 'b, CS> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, acid_io::Error> {
        // Safety: It only becomes uninit on drop
        let file = unsafe { self.file.assume_init_mut() };

        let mut fs = self.fs.borrow_mut();
        Ok(fs.read(file, buf))
    }
}
impl<'a, 'b, CS: hal::gpio::PinId> Drop for ReadableFile<'a, 'b, CS> {
    fn drop(&mut self) {
        let file = core::mem::replace(&mut self.file, MaybeUninit::uninit());
        let file = unsafe { file.assume_init() };
        let mut fs = self.fs.borrow_mut();
        fs.close(file);
    }
}
impl<'a, 'b, CS: hal::gpio::PinId> ReadableFile<'a, 'b, CS> {
    fn open(fs: &'a RefCell<sdcard::SDCardController<'b, CS>>, name: &str) -> Result<Self, ()> {
        let mut fs_ref = fs.borrow_mut();
        let file = MaybeUninit::new(fs_ref.open(name, Mode::ReadOnly)?);
        Ok(ReadableFile { file, fs })
    }
}

struct SpeakerControl {
    pin: hal::gpio::DynPin,
}

impl SpeakerControl {
    fn new(pin: impl Into<hal::gpio::DynPin>) -> Self {
        let mut ret = Self { pin: pin.into() };
        ret
    }

    fn off(&mut self) {
        self.pin.into_push_pull_output();
        self.pin.set_low().unwrap();
    }

    fn on(&mut self) {
        self.pin.into_floating_disabled();
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

    // Set up SPI for the id card reader:
    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio8.into_mode::<hal::gpio::FunctionSpi>();
    let spi_csn = pins.gpio9.into_push_pull_output();

    let spi: Spi<_, _, 8> = Spi::new(pac.SPI1).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut speaker_control = SpeakerControl::new(pins.gpio14);

    let mut id_reader = IdReader::new(spi, spi_csn);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut led_pin = pins.led.into_push_pull_output();

    output::setup_output(pac.PIO0, &mut timer, &mut pac.RESETS, pins.gpio15, cons);

    let mut card_poll_timer = timer.count_down();
    card_poll_timer.start(100.millis());

    let mut sd: sdcard::SDSpi<hal::gpio::pin::bank0::Gpio17> = sdcard::init_sd(
        pins.gpio18,
        pins.gpio19,
        pins.gpio16,
        pins.gpio17,
        pac.SPI0,
        &mut pac.RESETS,
        &clocks,
    );
    let fs = RefCell::new(sdcard::SDCardController::init(&mut sd));

    let mut data_fns = [data(100), data(200), data(400), data(800)];
    let mut data_fn_i = 0;
    let mut data = 0;
    let mut i = 0;

    let mut current_track: Option<claxon::FlacReader<ReadableFile<hal::gpio::pin::bank0::Gpio17>>> =
        None;

    // ----------------------------------------------------------------------------
    // Main loop! -----------------------------------------------------------------
    // ----------------------------------------------------------------------------

    let mut buf = alloc::vec::Vec::with_capacity(1024);
    let sin_test = false;
    loop {
        if current_track.is_none() || card_poll_timer.wait().is_ok() {
            if let Some(e) = id_reader.poll_event() {
                match e {
                    IdReaderEvent::New(id) => {
                        led_pin.set_high().unwrap();
                        let _ = current_track.take();

                        let file_name = id.filename_v2();
                        let Ok(file) = ReadableFile::open(&fs, &file_name) else {
                            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_3_SHORT);
                        };
                        let Ok(file) = claxon::FlacReader::new(file) else {
                            blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_2_SHORT);
                        };
                        speaker_control.on();

                        current_track = Some(file);
                    }
                    IdReaderEvent::Removed => {
                        led_pin.set_low().unwrap();

                        speaker_control.off();

                        current_track = None;
                    }
                }
            }
        }

        if let Some(ref mut file) = current_track {
            let mut blocks = file.blocks();
            let Ok(frame) = blocks.read_next_or_eof(core::mem::take(&mut buf)) else {
                    blink::blink_signals_loop(&mut led_pin, &mut delay, &blink::BLINK_ERR_3_SHORT);
                };
            let Some(frame) = frame else {
                    current_track = None;
                    continue;
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
                    ((v >> (config::FORMAT_BITS_PER_SAMPLE - config::BITS_PER_SAMPLE))
                        + config::ZERO_SAMPLE as i32) as u16
                };
                while prod.push(sample).is_err() {}
            }

            buf = frame.into_buffer();
        } else {
            //while prod.push(data).is_ok() {
            //    data = data_fns[data_fn_i].next().unwrap();
            //    i += 1;
            //    if i == 40000 {
            //        i = 0;
            //        data_fn_i = (data_fn_i + 1) % data_fns.len();
            //    }
            //}
        }
    }
}
