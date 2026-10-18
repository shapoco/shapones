# ShapoNES for the ClockworkPi PicoCalc

A port to the [ClockworkPi PicoCalc](https://www.clockworkpi.com/picocalc)
with a Raspberry Pi Pico 2 or Pico 2 W (RP2350). It builds on the XIAO
RP2350 port (`../xiao`): it reuses that port's FatFs, SD card driver and
lock/RAM/timer helpers, and takes the LCD set-up and colour table from
ClockworkPi's earlier PicoCalc patch for ShapoNES.

## What's PicoCalc-specific

- **LCD:** the 320x320 ST7365P on SPI1 at 62.5 MHz, RGB565. The 256x240
  picture is centred. No frame buffer is kept: each line goes out by DMA
  as soon as the PPU finishes it, which leaves RAM for ROMs up to 384 KB
  (loaded whole into RAM).
- **Keyboard:** the STM32 keyboard controller on I2C1 (address 0x1F),
  read as a FIFO of press/release events.
- **Sound:** PWM on GPIO 26 and 27 (the left and right speakers / headphone
  jack), both channels of PWM slice 5, 8-bit at 22.05 kHz.
- **SD card:** SPI0 (GPIO 16-19), which the card has to itself.
- **Boot menu:** runs from UF2 Loader; Ctrl+Esc returns to the PicoCalc
  Shell (`include/picocalc_shell.h`, MIT, by Blair Leduc).

## Keys

| Key | Button |
|---|---|
| Arrows | D-pad |
| `]` / X / K | A |
| `[` / Z / J | B |
| Enter / `=` | Start (Enter picks in the menu) |
| `-` / Space | Select |
| Esc | ShapoNES menu |
| Ctrl+Esc | back to the PicoCalc boot menu |

ROMs are read from `/nes` if it exists, otherwise from the root; the
list shows only folders and `.nes` files. Saved states go in `/nes-saves`.

## Building

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build
cmake -G Ninja ..
ninja
```

Copy `shapones_picocalc.uf2` to the SD card's `pico2-apps` folder.

## Tested

On a PicoCalc with a Pico 2 W: Super Mario Bros., Mega Man, The Legend of
Zelda and Wizardry play, with sound (saved states untested). Tetris
doesn't: its playfield isn't drawn.

Setting `NES_DEBUG_ROW` to 1 in `src/main.cpp` shows the buttons the
emulator receives, as a row of squares above the picture.
