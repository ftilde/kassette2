use fugit::ExtU32;

use pio::Instruction;
use pio::InstructionOperands;
use pio::OutDestination;
use pio_proc::pio_file;
use rp_pico::hal;
use rp_pico::hal::pac;

// Import pio crates
use hal::pio::SM0;
use rp_pico::hal::pio::Running;
use rp_pico::hal::pio::StateMachine;
use rp_pico::hal::pio::Tx;
use rp_pico::hal::pio::ValidStateMachine;
use rp_pico::hal::prelude::*;

pub fn setup_output<
    P: hal::gpio::PinId + hal::gpio::bank0::BankPinId,
    M: hal::gpio::PinMode + hal::gpio::ValidPinMode<P>,
>(
    pio: pac::PIO0,
    timer: pac::TIMER,
    resets: &mut pac::RESETS,
    output_pin: hal::gpio::Pin<P, M>,
    consumer_queue: crate::queue::QueueConsumer,
) {
    let (mut pio0, sm0, _, _, _) = pio.split(resets);

    // Create a pio program
    let program = pio_file!("./src/pwm.pio", select_program("pwm"));
    let installed = pio0.install(&program.program).unwrap();

    let _pio_pin = output_pin.into_mode::<hal::gpio::FunctionPio0>();
    let output_pin_id = P::DYN.num;

    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut sm, _, mut tx) = hal::pio::PIOBuilder::from_program(installed)
        .set_pins(output_pin_id, 1)
        .side_set_pin_base(output_pin_id)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .clock_divisor_fixed_point(1, 0)
        .autopull(true)
        .buffers(hal::pio::Buffers::OnlyTx) // Increase queue size to a whopping 8*4=32 samples
        .build(sm0);

    // Set pio pindir for gpio25
    sm.set_pindirs([(output_pin_id, hal::pio::PinDir::Output)]);

    // Start state machine
    let sm = sm.start();

    // Set period
    pio_pwm_set_period(sm, &mut tx, crate::MAX_VALUE);

    let mut timer = hal::Timer::new(timer, resets);
    let queue_fill_period = 50u32.micros(); //< 32 samples/44kHz
    crate::queue::setup_timer_interrupt(&mut timer, queue_fill_period, consumer_queue, tx);
}

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
