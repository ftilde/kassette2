use embedded_sdmmc::{
    filesystem::Mode, BlockSpi, Controller, Directory, File, SdMmcSpi, TimeSource, Timestamp,
    Volume, VolumeIdx,
};
use fugit::RateExtU32;
use rp_pico::{
    hal::{self, Clock},
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

pub struct SDCardController<'a> {
    controller: Controller<
        BlockSpi<
            'a,
            hal::Spi<hal::spi::Enabled, pac::SPI0, 8>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::Output<hal::gpio::PushPull>>,
        >,
        DummyTimesource,
    >,
    root_dir: Directory,
    volume: Volume,
}

impl SDCardController<'_> {
    pub fn open(&mut self, filename: &str, mode: Mode) -> File {
        self.controller
            .open_file_in_dir(&mut self.volume, &self.root_dir, filename, mode)
            .unwrap()
    }

    pub fn close(&mut self, file: File) {
        self.controller.close_file(&self.volume, file).unwrap();
    }

    pub fn read(&mut self, file: &mut File, buf: &mut [u8]) -> usize {
        self.controller.read(&self.volume, file, buf).unwrap()
    }

    pub fn init(sdspi: &mut SDSpi) -> SDCardController {
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

type SDSpi = SdMmcSpi<
    hal::Spi<hal::spi::Enabled, pac::SPI0, 8>,
    hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::Output<hal::gpio::PushPull>>,
>;

pub fn init_sd(
    _spi_sclk: hal::gpio::Pin<hal::gpio::bank0::Gpio18, hal::gpio::FunctionSpi>,
    _spi_mosi: hal::gpio::Pin<hal::gpio::bank0::Gpio19, hal::gpio::FunctionSpi>,
    _spi_miso: hal::gpio::Pin<hal::gpio::bank0::Gpio16, hal::gpio::FunctionSpi>,
    spi_csn: hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::PushPullOutput>,
    spi: pac::SPI0,
    resets: &mut pac::RESETS,
    clocks: &hal::clocks::ClocksManager,
) -> SDSpi {
    let spi = hal::spi::Spi::<_, _, 8>::new(spi);

    let spi = spi.init(
        resets,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    SdMmcSpi::new(spi, spi_csn)
}
