pub const BITS_PER_SAMPLE: u16 = 10;
pub const ZERO_SAMPLE: u16 = 1 << (BITS_PER_SAMPLE - 1);
pub const MAX_SAMPLE: u16 = (1 << BITS_PER_SAMPLE) - 1;

pub const SAMPLE_QUEUE_SIZE: usize = 4 << 10; // In KB
pub const HEAP_SIZE: usize = 64 << 10; // in KB
