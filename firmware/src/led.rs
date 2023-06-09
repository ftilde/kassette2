use core::cell::RefCell;

use embedded_hal::digital::v2::OutputPin;
use fugit::ExtU32;
use fugit::MicrosDurationU32;
use rp_pico::hal;

use cortex_m::interrupt::Mutex;
use hal::gpio::Pin;
use hal::gpio::PushPullOutput;
use hal::timer::Alarm;
use hal::timer::Alarm1;
use pac::interrupt;
use rp_pico::pac;

pub type LedPin = Pin<rp_pico::hal::gpio::pin::bank0::Gpio15, PushPullOutput>;

struct TimerIrqData {
    alarm: Alarm1,
    led: LedPin,
    current_sequence: BlinkSequence,
}

const MAX_SEQUENCE_LEN: usize = 3;
const COMMAND_MULTIPLIER_BASE_MILLIS: u32 = 10;

struct BlinkSequence {
    data: [i8; MAX_SEQUENCE_LEN],
}

impl BlinkSequence {
    fn new(seq: [i8; MAX_SEQUENCE_LEN]) -> Self {
        Self { data: seq }
    }
    fn none() -> Self {
        Self::new([0; MAX_SEQUENCE_LEN])
    }
    fn next(&mut self) -> Option<BlinkCommand> {
        let current_encoded = self.data[0];
        self.data = core::array::from_fn(|i| {
            if i == MAX_SEQUENCE_LEN - 1 {
                0
            } else {
                self.data[i + 1]
            }
        });

        let duration =
            ((current_encoded as i32).abs() as u32 * COMMAND_MULTIPLIER_BASE_MILLIS).millis();
        let state = match current_encoded.cmp(&0) {
            core::cmp::Ordering::Less => Some(LedState::Off),
            core::cmp::Ordering::Equal => None,
            core::cmp::Ordering::Greater => Some(LedState::On),
        };

        state.map(|state| BlinkCommand { duration, state })
    }
}

enum LedState {
    On,
    Off,
}

struct BlinkCommand {
    state: LedState,
    duration: MicrosDurationU32,
}

static TIMER_IRQ_DATA: Mutex<RefCell<Option<TimerIrqData>>> = Mutex::new(RefCell::new(None));

pub fn setup_timer_interrupt(timer: &mut hal::Timer, led: LedPin) {
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_1);
    }

    let mut alarm1 = timer.alarm_1().unwrap();
    alarm1.enable_interrupt();
    alarm1.schedule(1000u32.micros()).unwrap();
    cortex_m::interrupt::free(|cs| {
        TIMER_IRQ_DATA.borrow(cs).replace(Some(TimerIrqData {
            alarm: alarm1,
            led,
            current_sequence: BlinkSequence::none(),
        }));
    });
}

pub fn set_blink_sequence(seq: [i8; MAX_SEQUENCE_LEN]) {
    cortex_m::interrupt::free(|cs| {
        let data = TIMER_IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();
        data.current_sequence = BlinkSequence::new(seq);

        data.alarm.schedule(1000u32.micros()).unwrap();
    });
}

#[interrupt]
fn TIMER_IRQ_1() {
    cortex_m::interrupt::free(|cs| {
        let data = TIMER_IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        data.alarm.clear_interrupt();

        if let Some(c) = data.current_sequence.next() {
            match c.state {
                LedState::On => {
                    let _ = data.led.set_high();
                }
                LedState::Off => {
                    let _ = data.led.set_low();
                }
            }
            data.alarm.schedule(c.duration).unwrap();
        } else {
            let _ = data.led.set_low();
        }
    })
}
