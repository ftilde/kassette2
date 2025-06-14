# Kassette

![assembled kassette box](https://github.com/user-attachments/assets/ba009711-90cc-459f-99a2-4345d535614e)

This project aims to mimic the cassette recorder experience of the 80s/90s for the young generation:
Put an RFID-card on speaker-box and an audio drama/book starts playing.
The only other input modes are a volume knob and an on/off button.
If the card is removed, the playback is paused.
When placing (the same) card back on the box, playback resumes at the previous position.
Playing a different card resets the playback position of the previous card (i.e., only one playback position is stored).

The heart of kassette is a [raspberri pi pico](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#pico-1-family) which listens to events from the RFID-reader, reads [qoa](https://qoaformat.org/)-encoded files from an sd-card, and plays them using a custom "DAC" utilizing the [PIO](https://www.raspberrypi.com/news/what-is-pio/) cores of the pico.

## Building

A kassette box is cheap (about 30€) and comparatively easy to build.
The electronic components fit on a small board:
See `schematic.fzz` (open with [Fritzing](https://fritzing.org/)).
After completing the electronics, you can attach a speaker, status LED, button and potentiometer and fit those in a small wooden box.

![look inside kassette box](https://github.com/user-attachments/assets/cfecb9fc-51ab-4f26-8d83-08304370bb3c)
![closer look inside kassette box](https://github.com/user-attachments/assets/f7f63b26-8c18-43b0-bbbb-699c5b500501)

| Component                  | Price  |
| -------------------------- | ------ |
| wooden box                 | 2€     |
| usb charger+cable          | 4€     |
| speaker, 3W, 8Ohm          | 3€     |
| Rpi pico                   | 5€     |
| TDA 7267                   | 1.50€  |
| breadboard                 | 1€     |
| microsd breakout/adaptor   | 2.50€  |
| RFID reader (RFID RC522)   | 5€     |
| potentiometer (1kOhm)      | 1€     |
| a nice button              | 1€     |
| MicroSDHC 32GB             | 4€     |
| ceramic capacitor 100nF    | cents  |
| catalytic capacitor 0.1mF  | cents  |
| catalytic capacitor 4.7uF  | cents  |
| catalytic capacitor 0.47mF | cents  |
| resistor 15kOhm            | cents  |
| resistor 100Ohm            | cents  |
| LED                        | cents  |
| -------------------------- | ------ |
| total                      | ~30€   |

Additional you'll need RFID cards.
I bought 100 for about 30€.

## Adding media files

All content is stored on an sd-card.
The RFID-cards only select titles stored on that card.

### Prerequisites

- Install [rust](https://www.rust-lang.org/tools/install) with the `thumbv6m-none-eabi` toolchain
- Install elf2ufs: `cargo install --locked elf2uf2-rs`

### Preparing RFID-cards
If you don't know the ID of your RFID-cards, you can use the raspberry pi pico and RFID reader of the box to read them.
To do connect the box via USB cable (i.e., the power cable of the box), flash the `id_reader_firmware` and connect to the tty.

```sh
cd id_reader_firmware
make flash
make tty
```

Then place cards on the reader and note down the id.

### Adding media to the sd-card

Select one folder on your machine which acts as a mirror for media files on the sd-card.
Copy media files into the folder and then add them to a `media_definition.txt` file in the same folder.
The file contains one line per media file in the following format.
Comments are preceded by a \#.

```txt
# a comment
0x1d1d1d1d media_file.ogg
0x1bafbefa another_file.ogg
```

The sd-card must be formatted with a fat32 file system without a partition table.
Mount the file system and then execute the `sync_tool`:

```sh
cd sync_tool
cargo run --release <path_to_media_folder> <path_to_mounted_sdcard_fs>
```

### Flash the kassette firmware

Connect the raspberri pi pico via USB, navigate to the `firmware` subfolder and execute the flash command:
```sh
cd firmware
make flash
```

Now remove power to the kassette box, insert the sd-card and reapply power.
Place a card on the RFID reader and the associated audio drama should start playing.

## Older version

A previous version of kasette based on a raspberri pi and a hifiberry dac can be found [here](https://github.com/ftilde/kassette).
The sync tool can also be used with sd-cards for the older version, but there is no real reason to use it if you have not built it already.

## License

`kassette` is released under GPL 3.0 or later. See `COPYING` for details.
