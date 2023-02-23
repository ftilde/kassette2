use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

use rp_pico::hal;
use rp_pico::hal::pac;

// Import pio crates
use hal::pio::SM0;
use pac::interrupt;
use rp_pico::hal::timer::Alarm;
use rp_pico::hal::timer::Alarm0;

pub type SampleQueue = ringbuf::StaticRb<u8, { crate::config::SAMPLE_QUEUE_SIZE }>;
pub type QueueConsumer = ringbuf::consumer::Consumer<u8, &'static SampleQueue>;
//type QueueProducer = ringbuf::producer::Producer<[u8; 4], &'static SampleQueue>;
type PioTx = hal::pio::Tx<(pac::PIO0, SM0)>;

struct TimerIrqData {
    alarm: Alarm0,
    period: fugit::MicrosDurationU32,
    queue_input: QueueConsumer,
    pio_tx: PioTx,
    //led: LedPin,
}
static TIMER_IRQ_DATA: Mutex<RefCell<Option<TimerIrqData>>> = Mutex::new(RefCell::new(None));

pub fn setup_timer_interrupt(
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

        data.pio_tx.write(u32::from_le_bytes(d));
    }

    data.alarm.clear_interrupt();
    data.alarm.schedule(data.period).unwrap();
}
