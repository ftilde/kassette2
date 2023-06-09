#![no_std]
#![no_main]

mod led;
mod output;
mod panic;
mod queue;
mod sdcard;

use core::mem::ManuallyDrop;
use core::sync::atomic::AtomicU8;

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

// Pull in any important traits
use rp_pico::hal;
use rp_pico::hal::clocks::ClockSource;
use rp_pico::hal::clocks::StoppableClock;
use rp_pico::hal::multicore::Multicore;
use rp_pico::hal::pac;
use rp_pico::hal::pll::common_configs::PLL_SYS_125MHZ;
use rp_pico::hal::pll::setup_pll_blocking;
use rp_pico::hal::prelude::*;
use rp_pico::hal::Spi;

use fugit::RateExtU32;

use id_reader::*;
use rp_pico::hal::timer::Instant;

use crate::sdcard::SDCardFile;

//#[alloc_error_handler]
//fn oom(_: Layout) -> ! {
//    loop {}
//}

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

struct BorrowedMut<'a, T>(&'a mut T);

impl<'a, T: OutputPin<Error = core::convert::Infallible>> OutputPin for BorrowedMut<'a, T> {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()
    }
}

impl<'a, E, T: embedded_hal::blocking::spi::Transfer<u8, Error = E>>
    embedded_hal::blocking::spi::Transfer<u8> for BorrowedMut<'a, T>
{
    type Error = E;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.0.transfer(words)
    }
}
impl<'a, E, T: embedded_hal::blocking::spi::Write<u8, Error = E>>
    embedded_hal::blocking::spi::Write<u8> for BorrowedMut<'a, T>
{
    type Error = E;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.0.write(words)
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

#[repr(u8)]
#[derive(Copy, Clone)]
enum Core1State {
    Running,
    Stopped,
}

static CORE1_STATE_REQUEST: AtomicU8 = AtomicU8::new(Core1State::Running as _);
static CORE1_STATE_CURRENT: AtomicU8 = AtomicU8::new(Core1State::Running as _);

fn transition_core1_blocking(to: Core1State) {
    use core::sync::atomic::Ordering;
    CORE1_STATE_REQUEST.store(to as _, Ordering::Release);
    while CORE1_STATE_CURRENT.load(Ordering::Acquire) != to as _ {}
}

pub type CardEventQueue = ringbuf::StaticRb<IdReaderEvent, 4>;
pub type CardEventConsumer = ringbuf::consumer::Consumer<IdReaderEvent, &'static CardEventQueue>;
pub type CardEventProducer = ringbuf::producer::Producer<IdReaderEvent, &'static CardEventQueue>;

fn core1_task(
    mut producer: CardEventProducer,
    mut id_reader_spi: Spi<hal::spi::Enabled, pac::SPI1, 8>,
    mut spi_csn: impl OutputPin<Error = core::convert::Infallible>,
    mut id_reader_reset: hal::gpio::Pin<
        hal::gpio::bank0::Gpio7,
        hal::gpio::Output<hal::gpio::PushPull>,
    >,
) -> ! {
    let delay_cycles = 12500000; //100Ms with default clock rate

    loop {
        id_reader_reset.set_high().unwrap();
        let spi_csn = BorrowedMut(&mut spi_csn);
        let id_reader_spi = BorrowedMut(&mut id_reader_spi);
        let mut id_reader = IdReader::new(id_reader_spi, spi_csn);

        loop {
            if let Some(e) = id_reader.poll_event() {
                while producer.push(e).is_err() {}
            }

            if CORE1_STATE_REQUEST.load(Ordering::Acquire) == Core1State::Stopped as _ {
                // Go to sleep mode
                break;
            }
            cortex_m::asm::delay(delay_cycles);
        }
        // Disable card reader to save power
        id_reader_reset.set_low().unwrap();

        use core::sync::atomic::Ordering;
        CORE1_STATE_CURRENT.store(Core1State::Stopped as _, Ordering::Release);
        // Core 0 will transition to dormant mode now
        while CORE1_STATE_REQUEST.load(Ordering::Acquire) == Core1State::Stopped as _ {}
        CORE1_STATE_CURRENT.store(Core1State::Running as _, Ordering::Release);
    }
}

#[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
#[repr(C)]
struct SaveState {
    pos_millis: u32,
    id: Id,
}

const SAVE_STATE_FILE_NAME: &str = "save.bin";

fn load_save_state<CS: hal::gpio::PinId>(
    fs: &mut sdcard::SDCardController<CS>,
) -> Option<SaveState> {
    use acid_io::Read;
    let mut file = SDCardFile::open(fs, SAVE_STATE_FILE_NAME, Mode::ReadOnly).ok()?;
    let mut buffer = [0u8; core::mem::size_of::<SaveState>()];
    file.read_exact(&mut buffer).ok();

    Some(bytemuck::pod_read_unaligned(&buffer))
}

fn save_state<CS: hal::gpio::PinId>(
    fs: &mut sdcard::SDCardController<CS>,
    state: Option<SaveState>,
) {
    if let Some(state) = state {
        use acid_io::Write;
        let mut file =
            SDCardFile::open(fs, SAVE_STATE_FILE_NAME, Mode::ReadWriteCreateOrTruncate).unwrap();
        file.write_all(bytemuck::bytes_of(&state)).unwrap();
    } else {
        fs.try_unlink(SAVE_STATE_FILE_NAME);
    }
}

static mut CORE1_STACK: hal::multicore::Stack<4096> = hal::multicore::Stack::new();

fn run() -> ! {
    let mut rb = queue::SampleQueue::default();
    let (prod, cons) = rb.split_ref();

    let mut event_q = CardEventQueue::default();
    let (event_prod, event_cons) = event_q.split_ref();

    // Safety: Transmute to 'static lifetime. This is fine since main actually never returns.
    let (cons, mut prod, mut event_cons, event_prod) = unsafe {
        use core::mem::transmute;
        (
            transmute(cons),
            transmute(prod),
            transmute(event_prod),
            transmute(event_cons),
        )
    };

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
    let mut sio = hal::Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

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

    let id_reader_reset = pins.gpio7.into_push_pull_output();

    let id_reader_spi: Spi<_, _, 8> = Spi::new(pac.SPI1).init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        10.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut speaker_control = SpeakerControl::new(pins.gpio28);

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let output_pin = pins.gpio18;
    let mut sm = output::setup_output(pac.PIO0, &mut timer, &mut pac.RESETS, output_pin, cons);

    let led_pin = pins.gpio15.into_push_pull_output();
    let mut button_pin = pins.gpio5.into_pull_up_input();

    crate::led::setup_timer_interrupt(&mut timer, led_pin);

    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(event_prod, id_reader_spi, spi_csn, id_reader_reset)
    });

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

        led::set_blink_sequence(BLINK_SEQ_START);

        run_until_poweroff(
            &mut prod,
            &mut event_cons,
            &mut sd,
            &mut timer,
            &mut button_pin,
            &mut speaker_control,
        );
        speaker_control.off();
        sm = sm_s.stop();

        transition_core1_blocking(Core1State::Stopped);
        // Drain queue of potential old events after core 1 has stopped.
        while event_cons.pop().is_some() {}

        dormant_sleep_until_interrupt(&mut clocks, &mut button_pin);
        transition_core1_blocking(Core1State::Running);
    }
}

fn dormant_sleep_until_interrupt(
    clocks: &mut hal::clocks::ClocksManager,
    button_pin: &mut hal::gpio::Pin<hal::gpio::bank0::Gpio5, hal::gpio::Input<hal::gpio::PullUp>>,
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

    // Wait for button to turn off again
    while button_pin.is_low().unwrap() {}

    //    TODO: - see if some clocks can be skipped (and maybe disabled altogether)
}

type Reader<'a, 'b> = embedded_qoa::Reader<'a, SDCardFile<'a, 'b, hal::gpio::pin::bank0::Gpio1>>;
enum State<'a, 'b> {
    Stopping {
        id: Id,
        file: ManuallyDrop<Reader<'a, 'b>>,
        done_at: Instant,
    },
    Stopped {
        id: Id,
        file: ManuallyDrop<Reader<'a, 'b>>,
        since: Instant,
    },
    Starting {
        id: Id,
        file: ManuallyDrop<Reader<'a, 'b>>,
        since: Instant,
    },
    Started {
        id: Id,
        file: ManuallyDrop<Reader<'a, 'b>>,
    },
    Empty {
        since: Instant,
    },
}

impl<'a, 'b> State<'a, 'b> {
    fn stop(&mut self, timer: &hal::Timer) {
        use State::*;
        let fade_duration = TimerDurationU64::millis(config::FADE_DURATION_MILLIS);
        let now = timer.get_counter();
        let s = core::mem::replace(self, Empty { since: now });
        *self = match s {
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

    fn into_file(self) -> Option<(Id, Reader<'a, 'b>)> {
        if let State::Starting { file, id, .. }
        | State::Started { file, id, .. }
        | State::Stopped { file, id, .. }
        | State::Stopping { file, id, .. } = self
        {
            Some((id, ManuallyDrop::into_inner(file)))
        } else {
            None
        }
    }
}

const BLINK_SEQ_START: [i8; 3] = [30, 0, 0];
const BLINK_SEQ_END: [i8; 3] = [10, -10, 10];

fn run_until_poweroff(
    prod: &mut QueueProducer,
    event_consumer: &mut CardEventConsumer,
    sd: &mut sdcard::SDSpi<hal::gpio::bank0::Gpio1>,
    timer: &mut hal::Timer,
    button_pin: &mut dyn InputPin<Error = core::convert::Infallible>,
    speaker_control: &mut SpeakerControl,
) {
    let mut fs = sdcard::SDCardController::init(sd);

    const NUM_CHANNELS: u32 = 1;
    let mut frame_buffer = [0u8; embedded_qoa::max_frame_size(NUM_CHANNELS)];
    let mut sample_buffer = [0i16; embedded_qoa::max_sample_buffer_len(NUM_CHANNELS)];

    let mut state: State = if let Some(state) = load_save_state(&mut fs) {
        let file_name = state.id.filename_v2();
        let file = SDCardFile::open(&mut fs, &file_name, Mode::ReadOnly).unwrap();
        let mut file = Reader::new(file, &mut frame_buffer, &mut sample_buffer).unwrap();
        file.seek(core::time::Duration::from_millis(state.pos_millis as _))
            .unwrap();
        let file = ManuallyDrop::new(file);
        State::Stopped {
            id: state.id,
            file,
            since: timer.get_counter(),
        }
    } else {
        State::Empty {
            since: timer.get_counter(),
        }
    };

    // ----------------------------------------------------------------------------
    // Main loop! -----------------------------------------------------------------
    // ----------------------------------------------------------------------------

    let mut turn_off_pressed = false;

    let idle_sleep_time = TimerDurationU64::secs(config::IDLE_SLEEP_TIME_SECONDS);

    let fade_duration = TimerDurationU64::millis(config::FADE_DURATION_MILLIS);
    let context_seek = core::time::Duration::from_millis(2 * config::FADE_DURATION_MILLIS);
    loop {
        if let Some(e) = event_consumer.pop() {
            let now = timer.get_counter();
            match e {
                IdReaderEvent::New(n_id) => {
                    speaker_control.on();

                    state = match state {
                        State::Starting { id, file, since } if id == n_id => {
                            State::Starting { id, file, since }
                        }
                        State::Started { id, file } if id == n_id => State::Started { id, file },
                        State::Stopped {
                            id,
                            mut file,
                            since: _,
                        } if id == n_id => {
                            let seek_target = file.playback_position().saturating_sub(context_seek);
                            file.seek(seek_target).unwrap();

                            led::set_blink_sequence(BLINK_SEQ_START);
                            State::Starting {
                                id,
                                file,
                                since: now,
                            }
                        }
                        State::Stopping { id, file, done_at } if id == n_id => State::Starting {
                            id,
                            file,
                            since: now
                                - (done_at
                                    .checked_duration_since(now)
                                    .unwrap_or(TimerDurationU64::from_ticks(0))),
                        },
                        o => {
                            core::mem::drop(o.into_file());

                            // Old file dropped now
                            let file_name = n_id.filename_v2();
                            let file =
                                SDCardFile::open(&mut fs, &file_name, Mode::ReadOnly).unwrap();
                            let file =
                                Reader::new(file, &mut frame_buffer, &mut sample_buffer).unwrap();
                            //file.seek(core::time::Duration::from_secs(45 * 60)).unwrap();
                            let file = ManuallyDrop::new(file);

                            let now = timer.get_counter();

                            led::set_blink_sequence(BLINK_SEQ_START);
                            State::Starting {
                                id: n_id,
                                file,
                                since: now,
                            }
                        }
                    };
                }
                IdReaderEvent::Removed => {
                    state.stop(timer);
                }
            }
        }

        //TODO: Ideally we want an interrupt to handle this because we might miss the low state
        //otherwise
        if button_pin.is_low().unwrap() {
            state.stop(timer);
            turn_off_pressed = true;

            led::set_blink_sequence(BLINK_SEQ_END);
        }

        let now = timer.get_counter();
        if let State::Stopped { since, .. } | State::Empty { since } = &state {
            if turn_off_pressed || *since + idle_sleep_time < now {
                break;
            }
        }

        // Update state based on time
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
            let frame = file.read_frame().unwrap();

            let Some(frame) = frame else {
                // Drop current file
                let _ = state.into_file();
                state = State::Empty {
                    since: timer.get_counter(),
                };
                continue;
            };

            for v in frame {
                let raw = (*v >> (config::FORMAT_BITS_PER_SAMPLE - config::BITS_PER_SAMPLE)) as i32;
                let scaled = raw * fade_mult_num / fade_mult_denom;
                let sample = (scaled + config::ZERO_SAMPLE as i32) as u16;
                while prod.push(sample).is_err() {}
            }
        }
    }
    let s_state = state.into_file().map(|(id, mut f)| SaveState {
        pos_millis: f.playback_position().as_millis() as u32,
        id,
    });
    save_state(&mut fs, s_state);
}
