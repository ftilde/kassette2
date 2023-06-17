use crate::hal::pac;
use cortex_m::interrupt::Mutex;
use embedded_hal::digital::v2::InputPin;
use fugit::ExtU32;
use hal::pac::interrupt;
use rp_pico::hal::{
    self,
    gpio::Interrupt,
    timer::{Alarm, Alarm2},
};

// Some short-cuts to useful types
use core::cell::RefCell;
//use critical_section::Mutex;
use hal::gpio;

pub type ButtonPin = gpio::Pin<gpio::bank0::Gpio5, gpio::PullUpInput>;

#[derive(Copy, Clone)]
enum DebouncingState {
    Debouncing(ButtonState),
    Fixed(ButtonState),
}

#[derive(Copy, Clone)]
enum ButtonState {
    Pressed,
    Released,
}

struct IrqData {
    button: ButtonPin,
    alarm: Alarm2,
    state: DebouncingState,
    event_pressed: bool,
    event_released: bool,
}

static IRQ_DATA: Mutex<RefCell<Option<IrqData>>> = Mutex::new(RefCell::new(None));

pub fn clear() {
    cortex_m::interrupt::free(|cs| {
        let data = IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        data.event_pressed = false;
        data.event_released = false;
    });
}

pub fn was_pressed() -> bool {
    cortex_m::interrupt::free(|cs| {
        let data = IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        data.event_pressed
    })
}

pub fn is_released() -> bool {
    cortex_m::interrupt::free(|cs| {
        let data = IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        matches!(data.state, DebouncingState::Fixed(ButtonState::Released))
    })
}

pub fn setup_interrupt(timer: &mut hal::Timer, button: ButtonPin) {
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_2);
    }

    let mut alarm = timer.alarm_2().unwrap();
    alarm.enable_interrupt();

    cortex_m::interrupt::free(|cs| {
        IRQ_DATA.borrow(cs).replace(Some(IrqData {
            button,
            state: DebouncingState::Fixed(ButtonState::Released),
            alarm,
            event_pressed: false,
            event_released: false,
        }));

        let data = IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        data.button.set_interrupt_enabled(Interrupt::EdgeHigh, true);
        data.button.set_interrupt_enabled(Interrupt::EdgeLow, true);
    });
}

const DEBOUNCE_DURATION_MILLIS: u32 = 30;

#[interrupt]
fn IO_IRQ_BANK0() {
    cortex_m::interrupt::free(|cs| {
        let data = IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        if data.button.interrupt_status(Interrupt::EdgeLow) {
            if matches!(data.state, DebouncingState::Fixed(ButtonState::Released)) {
                let _ = data.alarm.schedule(DEBOUNCE_DURATION_MILLIS.millis());
                data.state = DebouncingState::Debouncing(ButtonState::Pressed);
            }
            data.button.clear_interrupt(Interrupt::EdgeLow);
        }

        if data.button.interrupt_status(Interrupt::EdgeHigh) {
            if matches!(data.state, DebouncingState::Fixed(ButtonState::Pressed)) {
                let _ = data.alarm.schedule(DEBOUNCE_DURATION_MILLIS.millis());
                data.state = DebouncingState::Debouncing(ButtonState::Released);
            }
            data.button.clear_interrupt(Interrupt::EdgeHigh);
        }
    });
}

#[interrupt]
fn TIMER_IRQ_2() {
    cortex_m::interrupt::free(|cs| {
        let data = IRQ_DATA.borrow(cs);
        let mut data = data.borrow_mut();
        let data = data.as_mut().unwrap();

        data.alarm.clear_interrupt();

        let current_state = if data.button.is_high().unwrap() {
            ButtonState::Released
        } else {
            ButtonState::Pressed
        };
        let target_state = match data.state {
            DebouncingState::Debouncing(s) => s,
            DebouncingState::Fixed(_) => panic!("We should not be debouncing this"),
        };
        for s in [current_state, target_state] {
            match s {
                ButtonState::Pressed => data.event_pressed = true,
                ButtonState::Released => data.event_released = true,
            }
        }
        data.state = DebouncingState::Fixed(current_state);
    })
}
