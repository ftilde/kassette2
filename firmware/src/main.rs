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
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_sdmmc::Mode;
use fugit::TimerDurationU64;
//use queue::QueueConsumer;
use queue::QueueProducer;
//use defmt::info;
//use defmt_rtt as _;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal;
use rp_pico::hal::clocks::ClockSource;
use rp_pico::hal::clocks::StoppableClock;
use rp_pico::hal::pac;
use rp_pico::hal::pll::common_configs::PLL_SYS_125MHZ;
use rp_pico::hal::pll::setup_pll_blocking;
use rp_pico::hal::prelude::*;
use rp_pico::hal::Spi;

use embedded_alloc::Heap;

use fugit::ExtU32;
use fugit::RateExtU32;

use id_reader::*;
use rp_pico::hal::spi::Enabled;
use rp_pico::hal::timer::Instant;

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
    fn open(
        fs: &'a RefCell<sdcard::SDCardController<'b, CS>>,
        name: &str,
    ) -> Result<Self, embedded_sdmmc::Error<embedded_sdmmc::SdMmcError>> {
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
        ret.off();
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
    run()
}

fn run() -> ! {
    // First thing: Initialize the allocator
    {
        static mut HEAP_MEM: [MaybeUninit<u8>; config::HEAP_SIZE] =
            [MaybeUninit::uninit(); config::HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, config::HEAP_SIZE) }
    }

    let mut rb = queue::SampleQueue::default();
    let (prod, cons) = rb.split_ref();

    // Safety: Transmute to 'static lifetime. This is fine since main actually never returns.
    let cons = unsafe { core::mem::transmute(cons) };
    let mut prod = unsafe { core::mem::transmute(prod) };

    let core = unsafe { pac::CorePeripherals::steal() };
    let mut pac = unsafe { pac::Peripherals::steal() };

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let mut clocks = hal::clocks::init_clocks_and_plls(
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

    let mut speaker_control = SpeakerControl::new(pins.gpio17);

    let mut id_reader = IdReader::new(spi, spi_csn);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut sm = output::setup_output(pac.PIO0, &mut timer, &mut pac.RESETS, pins.gpio16, cons);

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let mut led_pin = pins.led.into_push_pull_output();

    let mut button_pin = pins.gpio5.into_pull_up_input();

    let mut sd = sdcard::init_sd(
        pins.gpio2,
        pins.gpio3,
        pins.gpio0,
        pins.gpio1,
        pac.SPI0,
        &mut pac.RESETS,
        &clocks,
    );
    loop {
        let sm_s = sm.start();
        run_until_poweroff(
            &mut prod,
            &mut sd,
            &mut timer,
            &mut led_pin,
            &mut button_pin,
            &mut id_reader,
            &mut speaker_control,
            &mut delay,
        );
        speaker_control.off();
        sm = sm_s.stop();

        dormant_sleep_until_interrupt(&mut clocks, &mut button_pin);
    }
}

fn dormant_sleep_until_interrupt(
    clocks: &mut hal::clocks::ClocksManager,
    _button_pin: &mut hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::Input<hal::gpio::PullUp>>,
) {
    let mut pac = unsafe { pac::Peripherals::steal() };

    // Set up a minimal external clock for dormant mode
    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, rp_pico::XOSC_CRYSTAL_FREQ.Hz()).unwrap();
    clocks
        .reference_clock
        .configure_clock(&xosc, xosc.get_freq())
        .unwrap();
    clocks
        .system_clock
        .configure_clock(&xosc, xosc.get_freq())
        .unwrap();
    clocks.usb_clock.disable();
    clocks.adc_clock.disable();
    clocks.gpio_output0_clock.disable();
    clocks.gpio_output1_clock.disable();
    clocks.gpio_output2_clock.disable();
    clocks.gpio_output3_clock.disable();
    clocks.rtc_clock.disable();
    clocks.peripheral_clock.disable();
    const PLL_PWR_BITS: u32 = 0x0000002d;
    pac.PLL_USB.pwr.write(|w| unsafe { w.bits(PLL_PWR_BITS) });
    pac.PLL_SYS.pwr.write(|w| unsafe { w.bits(PLL_PWR_BITS) });

    // Set up wake up interrupt
    // Note: Clearing is not required since the level interrupts are not latched
    pac.IO_BANK0.dormant_wake_inte[0].write(|w| w.gpio5_level_low().set_bit());

    // Enter dormant mode (we return from this when the interrupt fires
    let xosc = unsafe { xosc.dormant() };

    let xosc = xosc.free();
    let xosc = hal::xosc::setup_xosc_blocking(xosc, rp_pico::XOSC_CRYSTAL_FREQ.Hz()).unwrap();

    let pll_sys = setup_pll_blocking(
        pac.PLL_SYS,
        xosc.operating_frequency(),
        PLL_SYS_125MHZ,
        clocks,
        &mut pac.RESETS,
    )
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pac.PLL_USB,
        xosc.operating_frequency(),
        hal::pll::common_configs::PLL_USB_48MHZ,
        clocks,
        &mut pac.RESETS,
    )
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb).unwrap();

    clocks.usb_clock.enable();
    clocks.adc_clock.enable();
    clocks.gpio_output0_clock.enable();
    clocks.gpio_output1_clock.enable();
    clocks.gpio_output2_clock.enable();
    clocks.gpio_output3_clock.enable();
    clocks.rtc_clock.enable();
    clocks.peripheral_clock.enable();

    //    TODO:
    //        - see if some clocks can be skipped (and maybe disabled altogether)
    //        - reset clock as was done in blogpost https://ghubcoder.github.io/posts/awaking-the-pico/
    //        - hope that this fixes the sound glitches
    //        - turn off reader, sdcard
}

fn run_until_poweroff(
    prod: &mut QueueProducer,
    sd: &mut sdcard::SDSpi<hal::gpio::bank0::Gpio1>,
    timer: &mut hal::Timer,
    led_pin: &mut dyn OutputPin<Error = core::convert::Infallible>,
    button_pin: &mut dyn InputPin<Error = core::convert::Infallible>,
    id_reader: &mut IdReader<
        Spi<Enabled, pac::SPI1, 8>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio9, hal::gpio::Output<hal::gpio::PushPull>>,
    >,
    speaker_control: &mut SpeakerControl,
    delay: &mut cortex_m::delay::Delay,
) {
    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)

    let mut card_poll_timer = timer.count_down();
    card_poll_timer.start(100.millis());

    let fs = RefCell::new(sdcard::SDCardController::init(sd));

    let mut data_fns = [data(100), data(200), data(400), data(800)];
    let mut data_fn_i = 0;
    //let mut data = 0;
    let mut i = 0;

    type Reader<'a, 'b> = claxon::FlacReader<ReadableFile<'a, 'b, hal::gpio::pin::bank0::Gpio1>>;
    enum State<'a, 'b> {
        Stopping {
            id: Id,
            file: Reader<'a, 'b>,
            done_at: Instant,
        },
        Stopped {
            id: Id,
            file: Reader<'a, 'b>,
            since: Instant,
        },
        Starting {
            id: Id,
            file: Reader<'a, 'b>,
            since: Instant,
        },
        Started {
            id: Id,
            file: Reader<'a, 'b>,
        },
        Empty {
            since: Instant,
        },
    }

    let mut state: State = State::Empty {
        since: timer.get_counter(),
    };

    // ----------------------------------------------------------------------------
    // Main loop! -----------------------------------------------------------------
    // ----------------------------------------------------------------------------

    let mut buf = alloc::vec::Vec::with_capacity(1024);
    let sin_test = false;

    let idle_sleep_time = TimerDurationU64::secs(config::IDLE_SLEEP_TIME_SECONDS);

    let fade_duration = TimerDurationU64::millis(config::FADE_DURATION_MILLIS);
    loop {
        if (matches!(state, State::Empty { .. } | State::Stopped { .. })
            || card_poll_timer.wait().is_ok())
            && !matches!(state, State::Stopping { .. })
        {
            if let Some(e) = id_reader.poll_event() {
                let now = timer.get_counter();
                match e {
                    IdReaderEvent::New(n_id) => {
                        led_pin.set_high().unwrap();

                        speaker_control.on();

                        state = match state {
                            State::Starting { id, file, since } if id == n_id => {
                                State::Starting { id, file, since }
                            }
                            State::Started { id, file } if id == n_id => {
                                State::Started { id, file }
                            }
                            State::Stopped { id, file, since: _ } if id == n_id => {
                                State::Starting {
                                    id,
                                    file,
                                    since: now,
                                }
                            }
                            State::Stopping { id, file, done_at } if id == n_id => {
                                State::Starting {
                                    id,
                                    file,
                                    since: now - (done_at - now),
                                }
                            }
                            _ => {
                                // Old file dropped now
                                let file_name = n_id.filename_v2();
                                let Ok(file) = ReadableFile::open(&fs, &file_name) else {
                                    blink::blink_signals_loop(led_pin, delay, &blink::BLINK_ERR_3_SHORT);
                                };
                                let Ok(file) = claxon::FlacReader::new(file) else {
                                    blink::blink_signals_loop(led_pin, delay, &blink::BLINK_ERR_2_SHORT);
                                };
                                State::Starting {
                                    id: n_id,
                                    file,
                                    since: now,
                                }
                            }
                        };
                    }
                    IdReaderEvent::Removed => {
                        led_pin.set_low().unwrap();

                        use State::*;
                        state = match state {
                            Starting { id, file, since } => Stopping {
                                id,
                                file,
                                done_at: now + (now - since),
                            },
                            Started { id, file } => Stopping {
                                id,
                                file,
                                done_at: now + fade_duration,
                            },
                            o => o,
                        };
                    }
                }
            }
        }

        let now = timer.get_counter();
        if let State::Stopped { since, .. } | State::Empty { since } = state {
            if since + idle_sleep_time < now {
                break;
            }
        }

        //TODO: Ideally we want an interrupt to handle this because we might miss the low state
        //otherwise
        if button_pin.is_low().unwrap() {
            break;
        }

        // Update state
        state = match state {
            State::Stopping { id, file, done_at } if done_at < now => {
                speaker_control.off();
                State::Stopped {
                    id,
                    file,
                    since: now,
                }
            }
            State::Starting { id, file, since } if since + fade_duration < now => {
                State::Started { id, file }
            }
            o => o,
        };

        let fade_mult_denom = config::FADE_DURATION_MILLIS;
        let (mut file, fade_mult_num) = match &mut state {
            State::Stopping { file, done_at, .. } => (
                Some(file),
                (*done_at - now).to_millis() * fade_mult_denom / config::FADE_DURATION_MILLIS,
            ),
            State::Starting { file, since, .. } => (
                Some(file),
                (now - *since).to_millis() * fade_mult_denom / config::FADE_DURATION_MILLIS,
            ),
            State::Started { file, .. } => (Some(file), fade_mult_denom),
            _ => (None, 0),
        };
        let fade_mult_num = fade_mult_num as i32;
        let fade_mult_denom = fade_mult_denom as i32;

        if let Some(ref mut file) = file {
            let mut blocks = file.blocks();
            let Ok(frame) = blocks.read_next_or_eof(core::mem::take(&mut buf)) else {
                    blink::blink_signals_loop(led_pin, delay, &blink::BLINK_ERR_3_SHORT);
                };
            let Some(frame) = frame else {
                state = State::Empty {
                    since: timer.get_counter(),
                };
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
                    let raw = v >> (config::FORMAT_BITS_PER_SAMPLE - config::BITS_PER_SAMPLE);
                    let scaled = raw * fade_mult_num / fade_mult_denom;
                    (scaled + config::ZERO_SAMPLE as i32) as u16
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
    led_pin.set_low().unwrap();

    // TODO actually power stuff off
}
