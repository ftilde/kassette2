//! # Pico PIO PWM Blink Example
//!
//! Fades the LED on a Pico board using the PIO peripheral with an pwm program.
//!
//! This will fade in the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! This example uses a few advance pio tricks such as side setting pins and instruction injection.
//!
//! See the `Cargo.toml` file for Copyright and license details. Except for the pio program which is subject to a different license.

#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use fugit::RateExtU32;
//use defmt::info;
//use defmt_rtt as _;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Import pio crates
use fugit::ExtU32;
use hal::pio::{PIOBuilder, Running, StateMachine, Tx, ValidStateMachine, SM0};
use pac::interrupt;
use pio::{Instruction, InstructionOperands, OutDestination};
use pio_proc::pio_file;
use rp_pico::hal::timer::Alarm;
use rp_pico::hal::timer::Alarm0;

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

//const BLINK_OK_LONG: [u8; 1] = [8u8];
//const BLINK_OK_SHORT_LONG: [u8; 4] = [1u8, 0u8, 6u8, 0u8];
//const BLINK_OK_SHORT_SHORT_LONG: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 6u8, 0u8];
const BLINK_ERR_2_SHORT: [u8; 4] = [1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_3_SHORT: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_4_SHORT: [u8; 8] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_5_SHORT: [u8; 10] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_6_SHORT: [u8; 12] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];

fn blink_signals(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) {
    for bit in sig {
        if *bit != 0 {
            pin.set_high().unwrap();
        } else {
            pin.set_low().unwrap();
        }

        let length = if *bit > 0 { *bit } else { 1 };

        for _ in 0..length {
            delay.delay_ms(200);
        }
    }

    pin.set_low().unwrap();

    delay.delay_ms(500);
}

fn blink_signals_loop(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) -> ! {
    loop {
        blink_signals(pin, delay, sig);
        delay.delay_ms(1000);
    }
}

/// Set pio pwm period
///
/// This uses a sneaky trick to set a second value besides the duty cycle.
/// We first write a value to the tx fifo. But instead of the normal instructions we
/// have stopped the state machine and inject our own instructions that move the written value to the ISR.
fn pio_pwm_set_period<T: ValidStateMachine>(
    sm: StateMachine<(hal::pac::PIO0, SM0), Running>,
    tx: &mut Tx<T>,
    period: u32,
) -> StateMachine<(hal::pac::PIO0, SM0), Running> {
    // To make sure the inserted instructions actually use our newly written value
    // We first busy loop to empty the queue. (Which typically should be the case)
    while !tx.is_empty() {}

    let mut sm = sm.stop();
    tx.write(period);
    sm.exec_instruction(Instruction {
        operands: InstructionOperands::PULL {
            if_empty: false,
            block: false,
        },
        delay: 0,
        side_set: None,
    });
    sm.exec_instruction(Instruction {
        operands: InstructionOperands::OUT {
            destination: OutDestination::ISR,
            bit_count: 32,
        },
        delay: 0,
        side_set: None,
    });
    sm.start()
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
    let mut rb = SampleQueue::default();
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

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Create a pio program
    let program = pio_file!("./src/pwm.pio", select_program("pwm"));
    let installed = pio0.install(&program.program).unwrap();

    //let led: LedPin = pins.led.into_push_pull_output();

    //let output_pin_id = 25; //led

    // Set pin to pio
    let _pio_pin: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.gpio15.into_mode();
    let output_pin_id = 15; //led

    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
        .set_pins(output_pin_id, 1)
        .side_set_pin_base(output_pin_id)
        .clock_divisor_fixed_point(1, 0)
        .autopull(true)
        .buffers(hal::pio::Buffers::OnlyTx) // Increase queue size to a whopping 8*4=32 samples
        .build(sm0);

    // Set pio pindir for gpio25
    sm.set_pindirs([(output_pin_id, hal::pio::PinDir::Output)]);

    // Start state machine
    let sm = sm.start();

    // Set period
    pio_pwm_set_period(sm, &mut tx, MAX_VALUE);

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
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_2_SHORT);
        }
    };

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut cont = Controller::new(block, DummyTimesource::default());

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    match cont.device().card_size_bytes() {
        Ok(_size) => {}
        Err(_e) => {
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_3_SHORT);
        }
    }

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut volume = match cont.get_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(_e) => {
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_4_SHORT);
        }
    };

    //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    // After we have the volume (partition) of the drive we got to open the
    // root directory:
    let dir = match cont.open_root_dir(&volume) {
        Ok(dir) => dir,
        Err(_e) => {
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_5_SHORT);
        }
    };

    let mut file = match cont.open_file_in_dir(&mut volume, &dir, "bibi.bin", Mode::ReadOnly) {
        Ok(dir) => dir,
        Err(_e) => {
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_6_SHORT);
        }
    };

    // ----------------------------------------------------------------------------
    // Queue stuff ----------------------------------------------------------------
    // ----------------------------------------------------------------------------

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let queue_fill_period = 700.micros(); //< 32 samples/44kHz
    setup_timer_interrupt(&mut timer, queue_fill_period, cons, tx);

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
const SAMPLE_QUEUE_SIZE: usize = 1024;
type SampleQueue = ringbuf::StaticRb<u8, SAMPLE_QUEUE_SIZE>;
type QueueConsumer = ringbuf::consumer::Consumer<u8, &'static SampleQueue>;
//type QueueProducer = ringbuf::producer::Producer<[u8; 4], &'static SampleQueue>;
type PioTx = hal::pio::Tx<(pac::PIO0, SM0)>;
//type LedPin = hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::Output<hal::gpio::PushPull>>;

struct TimerIrqData {
    alarm: Alarm0,
    period: fugit::MicrosDurationU32,
    queue_input: QueueConsumer,
    pio_tx: PioTx,
    //led: LedPin,
}
static TIMER_IRQ_DATA: Mutex<RefCell<Option<TimerIrqData>>> = Mutex::new(RefCell::new(None));

fn setup_timer_interrupt(
    timer: &mut hal::Timer,
    period: fugit::MicrosDurationU32,
    queue_input: QueueConsumer,
    pio_tx: PioTx,
    //led: LedPin,
) {
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();
    alarm0.schedule(period).unwrap();
    cortex_m::interrupt::free(|cs| {
        TIMER_IRQ_DATA.borrow(cs).replace(Some(TimerIrqData {
            alarm: alarm0,
            period,
            queue_input,
            pio_tx,
            //led,
        }));
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndButton>`
    static mut DATA: Option<TimerIrqData> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    let data = if let Some(data) = DATA {
        data
    } else {
        cortex_m::interrupt::free(|cs| {
            *DATA = TIMER_IRQ_DATA.borrow(cs).take();
            DATA.as_mut().unwrap()
        })
    };

    while !data.pio_tx.is_full() {
        let mut d = [127u8; 4];
        data.queue_input.pop_slice(&mut d);
        data.pio_tx.write(bytemuck::cast(d));
    }

    data.alarm.clear_interrupt();
    data.alarm.schedule(data.period).unwrap();
}
