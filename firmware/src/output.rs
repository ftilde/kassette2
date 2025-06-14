use pio_proc::pio_file;
use rp_pico::hal;
use rp_pico::hal::pac;

// Import pio crates
use rp_pico::hal::prelude::*;

pub type PioStateMachine = hal::pio::StateMachine<(pac::PIO0, hal::pio::SM0), hal::pio::Stopped>;

pub fn setup_output<
    P: hal::gpio::PinId + hal::gpio::bank0::BankPinId,
    M: hal::gpio::PinMode + hal::gpio::ValidPinMode<P>,
>(
    pio: pac::PIO0,
    timer: &mut hal::Timer,
    resets: &mut pac::RESETS,
    output_pin: hal::gpio::Pin<P, M>,
    consumer_queue: crate::queue::QueueConsumer,
) -> PioStateMachine {
    let (mut pio0, sm0, _, _, _) = pio.split(resets);

    // Create a pio program
    let program = pio_file!("./src/pwm.pio", select_program("pwm"));
    let installed = pio0.install(&program.program).unwrap();

    let _pio_pin = output_pin.into_mode::<hal::gpio::FunctionPio0>();
    let output_pin_id = P::DYN.num;

    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut sm, _, tx) = hal::pio::PIOBuilder::from_program(installed)
        .set_pins(output_pin_id, 1)
        .side_set_pin_base(output_pin_id)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .clock_divisor_fixed_point(1, 0)
        .autopull(true)
        .buffers(hal::pio::Buffers::OnlyTx) // Increase queue size to a whopping 8*4=32 samples
        .build(sm0);

    // Set pio pindir for gpio25
    sm.set_pindirs([(output_pin_id, hal::pio::PinDir::Output)]);

    crate::queue::setup_timer_interrupt(timer, consumer_queue, tx);
    sm
}
