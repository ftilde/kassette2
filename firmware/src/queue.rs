use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use fugit::ExtU32;
use rp_pico::hal;
use rp_pico::hal::pac;

// Import pio crates
use hal::pio::SM0;
use pac::interrupt;
use rp_pico::hal::timer::Alarm;
use rp_pico::hal::timer::Alarm0;

pub type SampleQueue = ringbuf::StaticRb<u16, { config::SAMPLE_QUEUE_SIZE }>;
pub type QueueConsumer = ringbuf::consumer::Consumer<u16, &'static SampleQueue>;
pub type QueueProducer = ringbuf::producer::Producer<u16, &'static SampleQueue>;
type PioTx = hal::pio::Tx<(pac::PIO0, SM0)>;

struct TimerIrqData {
    alarm: Alarm0,
    queue_input: QueueConsumer,
    pio_tx: PioTx,
}
static TIMER_IRQ_DATA: Mutex<RefCell<Option<TimerIrqData>>> = Mutex::new(RefCell::new(None));

pub fn setup_timer_interrupt(timer: &mut hal::Timer, queue_input: QueueConsumer, pio_tx: PioTx) {
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    let mut alarm0 = timer.alarm_0().unwrap();
    alarm0.enable_interrupt();
    alarm0.schedule(1000u32.micros()).unwrap();
    cortex_m::interrupt::free(|cs| {
        TIMER_IRQ_DATA.borrow(cs).replace(Some(TimerIrqData {
            alarm: alarm0,
            queue_input,
            pio_tx,
        }));
    });
}

#[interrupt]
fn TIMER_IRQ_0() {
    static mut DATA: Option<TimerIrqData> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `DATA`.
    let data = if let Some(data) = DATA {
        data
    } else {
        cortex_m::interrupt::free(|cs| {
            *DATA = TIMER_IRQ_DATA.borrow(cs).take();
            DATA.as_mut().unwrap()
        })
    };

    while !data.pio_tx.is_full() {
        let mut d = [config::ZERO_SAMPLE; 2];
        data.queue_input.pop_slice(&mut d);

        let d = d[0] as u32 | ((d[1] as u32) << 16);

        data.pio_tx.write(d);
    }

    data.alarm.clear_interrupt();
    data.alarm
        .schedule(config::QUEUE_FILL_PERIOD_US.micros())
        .unwrap();
}
