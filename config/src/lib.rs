#![no_std]

// ----------------------------------------------------------------------------
// Configurable
// ----------------------------------------------------------------------------
pub const BITS_PER_SAMPLE: u16 = 10; //Also change in pwm.pio
pub const NUM_PWM_ITER: u16 = 3; //Also change in pwm.pio
pub const SAMPLE_QUEUE_SIZE_KB: usize = 4;
pub const HEAP_SIZE_KB: usize = 64;

// ----------------------------------------------------------------------------
// Should probably not be changed:
// ----------------------------------------------------------------------------
pub const FORMAT_BITS_PER_SAMPLE: u16 = 16;
pub const CLOCK_RATE: u32 = 125_000_000;
pub const QUEUE_SIZE: u32 = 16; // 32 Bytes, but 2 bytes per sample

// ----------------------------------------------------------------------------
// Derived from other values:
// ----------------------------------------------------------------------------
pub const SAMPLE_QUEUE_SIZE: usize = SAMPLE_QUEUE_SIZE_KB << 10;
pub const HEAP_SIZE: usize = HEAP_SIZE_KB << 10;

pub const ZERO_SAMPLE: u16 = 1 << (BITS_PER_SAMPLE - 1);
pub const MAX_SAMPLE: u16 = (1 << BITS_PER_SAMPLE) - 1;

pub const INSTRUCTIONS_PER_SAMPLE: u16 =
    ((1 << BITS_PER_SAMPLE/*Both tight loops*/) + 7) * NUM_PWM_ITER + 3;
pub const SAMPLE_RATE: u32 = CLOCK_RATE / INSTRUCTIONS_PER_SAMPLE as u32;
pub const QUEUE_FILL_PERIOD_US: u32 = 1_000_000 /*mu*/ * QUEUE_SIZE / SAMPLE_RATE /*Hz*/;
