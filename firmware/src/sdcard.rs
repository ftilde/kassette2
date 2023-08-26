use core::mem::MaybeUninit;

use acid_io::SeekFrom;
use embedded_sdmmc::{
    filesystem::Mode, Directory, File, SdCard, TimeSource, Timestamp, Volume, VolumeIdx,
    VolumeManager,
};
use fugit::RateExtU32;
use rp_pico::{
    hal::{
        self,
        gpio::{bank0::BankPinId, FunctionSpi, Pin, PinId, PinMode, ValidPinMode},
        Clock,
    },
    pac,
};

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

pub struct SDCardController<'a, CS: PinId> {
    controller: VolumeManager<SDCardBlockDeviceMut<'a, CS>, DummyTimesource>,
    root_dir: Directory,
    volume: Volume,
}

impl<'a, CS: PinId> SDCardController<'a, CS> {
    pub fn open(
        &mut self,
        filename: &str,
        mode: Mode,
    ) -> Result<File, embedded_sdmmc::Error<embedded_sdmmc::sdcard::Error>> {
        self.controller
            .open_file_in_dir(&mut self.volume, &self.root_dir, filename, mode)
    }

    pub fn try_unlink(&mut self, path: &str) {
        let _ = self
            .controller
            .delete_file_in_dir(&self.volume, &self.root_dir, path);
    }

    pub fn close(&mut self, file: File) {
        self.controller.close_file(&self.volume, file).unwrap();
    }

    pub fn read(&mut self, file: &mut File, buf: &mut [u8]) -> usize {
        self.controller.read(&self.volume, file, buf).unwrap()
    }

    pub fn write(&mut self, file: &mut File, buf: &[u8]) -> usize {
        self.controller.write(&mut self.volume, file, buf).unwrap()
    }

    pub fn init(sdspi: SDCardBlockDeviceMut<'a, CS>) -> SDCardController<CS> {
        // Next we need to aquire the block device and initialize the
        // communication with the SD card.
        //let block = sdspi.acquire().unwrap();

        let mut manager = VolumeManager::new(sdspi, DummyTimesource::default());

        let volume = manager.get_volume(VolumeIdx(0)).unwrap();

        //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

        // After we have the volume (partition) of the drive we got to open the
        // root directory:
        let root_dir = manager.open_root_dir(&volume).unwrap();
        SDCardController {
            controller: manager,
            volume,
            root_dir,
        }
    }
}

pub struct BlockingDelay {}
impl embedded_hal::blocking::delay::DelayUs<u8> for BlockingDelay {
    fn delay_us(&mut self, us: u8) {
        let mult = 125; // 125MHz * 1us = 125
        cortex_m::asm::delay(mult * (us as u32));
    }
}

pub type SDCardBlockDevice<CS> = SdCard<
    hal::Spi<hal::spi::Enabled, pac::SPI0, 8>,
    Pin<CS, hal::gpio::Output<hal::gpio::PushPull>>,
    BlockingDelay,
>;

pub struct SDCardBlockDeviceMut<'a, CS: PinId> {
    pub inner: &'a mut SDCardBlockDevice<CS>,
}

impl<'a, CS: PinId> embedded_sdmmc::BlockDevice for SDCardBlockDeviceMut<'a, CS> {
    type Error = embedded_sdmmc::sdcard::Error;

    fn read(
        &self,
        blocks: &mut [embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
        reason: &str,
    ) -> Result<(), Self::Error> {
        self.inner.read(blocks, start_block_idx, reason)
    }

    fn write(
        &self,
        blocks: &[embedded_sdmmc::Block],
        start_block_idx: embedded_sdmmc::BlockIdx,
    ) -> Result<(), Self::Error> {
        self.inner.write(blocks, start_block_idx)
    }

    fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
        self.inner.num_blocks()
    }
}

pub fn init_sd<
    CLK: PinId + BankPinId,
    MCLK: PinMode + ValidPinMode<CLK>,
    MOSI: PinId + BankPinId,
    MMOSI: PinMode + ValidPinMode<MOSI>,
    MISO: PinId + BankPinId,
    MMISO: PinMode + ValidPinMode<MISO>,
    CS: PinId,
    MCS: PinMode + ValidPinMode<CS>,
>(
    spi_sclk: Pin<CLK, MCLK>,
    spi_mosi: Pin<MOSI, MMOSI>,
    spi_miso: Pin<MISO, MMISO>,
    spi_csn: Pin<CS, MCS>,
    spi: pac::SPI0,
    resets: &mut pac::RESETS,
    clocks: &hal::clocks::ClocksManager,
) -> SDCardBlockDevice<CS> {
    let _spi_sclk = spi_sclk.into_mode::<FunctionSpi>();
    let _spi_mosi = spi_mosi.into_mode::<FunctionSpi>();
    let _spi_miso = spi_miso.into_mode::<FunctionSpi>();

    let spi = hal::spi::Spi::<_, _, 8>::new(spi);

    let spi = spi.init(
        resets,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    SDCardBlockDevice::new(spi, spi_csn.into_mode(), BlockingDelay {})
}

pub struct SDCardFile<'a, 'b, CS: hal::gpio::PinId> {
    fs: &'a mut SDCardController<'b, CS>,
    file: MaybeUninit<embedded_sdmmc::File>,
}

impl<'a, 'b, CS: hal::gpio::PinId> acid_io::Read for SDCardFile<'a, 'b, CS> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, acid_io::Error> {
        // Safety: It only becomes uninit on drop
        let file = unsafe { self.file.assume_init_mut() };

        Ok(self.fs.read(file, buf))
    }
}

impl<'a, 'b, CS: hal::gpio::PinId> acid_io::Write for SDCardFile<'a, 'b, CS> {
    fn write(&mut self, src: &[u8]) -> acid_io::Result<usize> {
        // Safety: It only becomes uninit on drop
        let mut file = unsafe { self.file.assume_init_mut() };
        Ok(self.fs.write(&mut file, src))
    }

    fn flush(&mut self) -> acid_io::Result<()> {
        // Nop. embedded_sdmmc always writes stuff out.
        Ok(())
    }
}

impl<'a, 'b, CS: hal::gpio::PinId> acid_io::Seek for SDCardFile<'a, 'b, CS> {
    fn seek(&mut self, pos: SeekFrom) -> acid_io::Result<u64> {
        // Safety: It only becomes uninit on drop
        let mut file = unsafe { self.file.assume_init_mut() };

        let res = match pos {
            SeekFrom::End(n) => file.seek_from_end(n as u32),
            SeekFrom::Start(n) => {
                const WARMING_READ_SIZE: usize = 1;
                let seek_target = n.saturating_sub(WARMING_READ_SIZE as _);
                let res = file.seek_from_start(seek_target as u32);
                let read_size = n - seek_target;
                let mut dummy_buffer = [0u8; WARMING_READ_SIZE];
                self.fs
                    .read(&mut file, &mut dummy_buffer[..read_size as usize]);
                res
            }
            SeekFrom::Current(n) => file.seek_from_current(n as i32),
        };
        if let Err(embedded_sdmmc::filesystem::FileError::InvalidOffset) = res {
            return Err(acid_io::ErrorKind::InvalidInput.into());
        }
        Ok((file.length() - file.left()) as u64)
    }
}

impl<'a, 'b, CS: hal::gpio::PinId> core::fmt::Write for SDCardFile<'a, 'b, CS> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Safety: It only becomes uninit on drop
        let file = unsafe { self.file.assume_init_mut() };

        let mut s = s.as_bytes();
        loop {
            let written = self.fs.write(file, s);
            s = &s[written..];
            if s.is_empty() {
                return Ok(());
            }
        }
    }
}

impl<'a, 'b, CS: hal::gpio::PinId> Drop for SDCardFile<'a, 'b, CS> {
    fn drop(&mut self) {
        let file = core::mem::replace(&mut self.file, MaybeUninit::uninit());
        let file = unsafe { file.assume_init() };
        self.fs.close(file);
    }
}
impl<'a, 'b, CS: hal::gpio::PinId> SDCardFile<'a, 'b, CS> {
    pub fn open(
        fs: &'a mut SDCardController<'b, CS>,
        name: &str,
        mode: Mode,
    ) -> Result<Self, embedded_sdmmc::Error<embedded_sdmmc::sdcard::Error>> {
        let file = MaybeUninit::new(fs.open(name, mode)?);
        Ok(SDCardFile { file, fs })
    }
}
