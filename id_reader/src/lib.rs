#![no_std]

use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Id(pub [u8; 4]);

fn fmt_nibble(nibble: u8) -> u8 {
    if nibble < 10 {
        nibble + b'0'
    } else {
        nibble - 10 + b'A'
    }
}

fn fmt_byte_to_nibbles(v: u8) -> [u8; 2] {
    let n_high = v >> 4;
    let n_low = v & 0xf;
    [fmt_nibble(n_high), fmt_nibble(n_low)]
}

impl core::fmt::Display for Id {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in &self.0 {
            let nibbles = fmt_byte_to_nibbles(*b);
            f.write_str(unsafe { core::str::from_utf8_unchecked(&nibbles) })?;
            write!(f, " ")?;
        }
        Ok(())
    }
}

pub struct FileNameStr {
    buf: [u8; 12],
}

impl core::ops::Deref for FileNameStr {
    type Target = str;

    fn deref(&self) -> &Self::Target {
        unsafe { core::str::from_utf8_unchecked(&self.buf) }
    }
}

impl Id {
    pub fn filename_v2(&self) -> FileNameStr {
        let mut buf = [0u8; 12];
        let mut i = 0;
        for b in &self.0 {
            let nibbles = fmt_byte_to_nibbles(*b);
            buf[i] = nibbles[0];
            buf[i + 1] = nibbles[1];
            i += 2;
        }
        for b in b".FLC" {
            buf[i] = *b;
            i += 1;
        }
        FileNameStr { buf }
    }
}

pub struct IdReader<SPI, NSS> {
    prev_state: Option<Id>,
    device: mfrc522::Mfrc522<SPI, NSS>,
}

pub enum IdReaderEvent {
    New(Id),
    Removed,
}

fn unpack_uid(v: mfrc522::Uid) -> Id {
    let v: [u8; 4] = v.as_bytes().try_into().unwrap();
    Id(v)
}

impl<E, SPI, NSS> IdReader<SPI, NSS>
where
    E: core::fmt::Debug,
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    NSS: OutputPin,
{
    pub fn new(spi: SPI, nss: NSS) -> Self {
        let device = mfrc522::Mfrc522::with_nss(spi, nss).unwrap();
        Self {
            device,
            prev_state: None,
        }
    }
    pub fn poll_event(&mut self) -> Option<IdReaderEvent> {
        let prev_id = self.prev_state.take();
        // For some reason, the second `reqa` after a `select` does not work if the card stays on
        // the reader. We thus retry exactly once here.
        if let Ok(a) = self.device.reqa().or_else(|_| self.device.reqa()) {
            if let Ok(n_uid) = self.device.select(&a) {
                let n_id = unpack_uid(n_uid);
                self.prev_state = Some(n_id);
                return match prev_id {
                    Some(id) if id == n_id => None,
                    Some(_) | None => Some(IdReaderEvent::New(n_id)),
                };
            }
        }
        self.prev_state = None;
        match prev_id {
            Some(_) => Some(IdReaderEvent::Removed),
            None => None,
        }
    }
}
