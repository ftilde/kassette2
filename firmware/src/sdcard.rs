use core::{cell::RefCell, mem::MaybeUninit};

use embedded_sdmmc::{
    filesystem::Mode, BlockSpi, Controller, Directory, File, SdMmcSpi, TimeSource, Timestamp,
    Volume, VolumeIdx,
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
    controller: Controller<
        BlockSpi<
            'a,
            hal::Spi<hal::spi::Enabled, pac::SPI0, 8>,
            Pin<CS, hal::gpio::Output<hal::gpio::PushPull>>,
        >,
        DummyTimesource,
    >,
    root_dir: Directory,
    volume: Volume,
}

impl<CS: PinId> SDCardController<'_, CS> {
    pub fn open(
        &mut self,
        filename: &str,
        mode: Mode,
    ) -> Result<File, embedded_sdmmc::Error<embedded_sdmmc::SdMmcError>> {
        self.controller
            .open_file_in_dir(&mut self.volume, &self.root_dir, filename, mode)
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

    pub fn init(sdspi: &mut SDSpi<CS>) -> SDCardController<CS> {
        // Next we need to aquire the block device and initialize the
        // communication with the SD card.
        let block = sdspi.acquire().unwrap();

        let mut controller = Controller::new(block, DummyTimesource::default());

        let volume = controller.get_volume(VolumeIdx(0)).unwrap();

        //blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

        // After we have the volume (partition) of the drive we got to open the
        // root directory:
        let root_dir = controller.open_root_dir(&volume).unwrap();
        SDCardController {
            controller,
            volume,
            root_dir,
        }
    }
}

pub type SDSpi<CS> = SdMmcSpi<
    hal::Spi<hal::spi::Enabled, pac::SPI0, 8>,
    Pin<CS, hal::gpio::Output<hal::gpio::PushPull>>,
>;

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
) -> SDSpi<CS> {
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

    SdMmcSpi::new(spi, spi_csn.into_mode())
}

pub struct SDCardFile<'a, 'b, CS: hal::gpio::PinId> {
    fs: &'a RefCell<SDCardController<'b, CS>>,
    file: MaybeUninit<embedded_sdmmc::File>,
}

impl<'a, 'b, CS: hal::gpio::PinId> acid_io::Read for SDCardFile<'a, 'b, CS> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, acid_io::Error> {
        // Safety: It only becomes uninit on drop
        let file = unsafe { self.file.assume_init_mut() };

        let mut fs = self.fs.borrow_mut();
        Ok(fs.read(file, buf))
    }
}

impl<'a, 'b, CS: hal::gpio::PinId> core::fmt::Write for SDCardFile<'a, 'b, CS> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Safety: It only becomes uninit on drop
        let file = unsafe { self.file.assume_init_mut() };

        let mut fs = self.fs.borrow_mut();
        let mut s = s.as_bytes();
        loop {
            let written = fs.write(file, s);
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
        let mut fs = self.fs.borrow_mut();
        fs.close(file);
    }
}
impl<'a, 'b, CS: hal::gpio::PinId> SDCardFile<'a, 'b, CS> {
    pub fn open(
        fs: &'a RefCell<SDCardController<'b, CS>>,
        name: &str,
        mode: Mode,
    ) -> Result<Self, embedded_sdmmc::Error<embedded_sdmmc::SdMmcError>> {
        let mut fs_ref = fs.borrow_mut();
        let file = MaybeUninit::new(fs_ref.open(name, mode)?);
        Ok(SDCardFile { file, fs })
    }
}
