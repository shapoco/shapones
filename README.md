# ShapoNES

an NES emulator for my programming study.

## Raspberry Pi Pico

a sample using WAVESHARE-19804, supporting NES loading from SD card.

![](img/bootmenu.jpg)
![](img/cover.jpg)
![](img/backside.jpg)
![](img/circuit.png)

## PC (wxWidgets)

a sample for PCs using wxWidgets.

![](img/ss01.png)

### How to Build/Run (WSL2)

1. Install PulseAudio on Windows.
2. Install required libraries on WSL2.

    ```sh
    sudo apt update
    sudo apt install libwxgtk3.2-dev pulseaudio libpulse-dev
    ```

3. Build and run.

    ```sh
    git clone https://github.com/shapoco/shapones.git
    pushd shapones/roms
      wget http://nickmass.com/images/nestest.nes
    popd
    pushd shapones/samples/wxapp
      make -j
      make run
    popd
    ```
