# Codec2 Lora Gateway

**Attention:** This project is in a very early alpha state. It lacks in some basic features and in documatation.

## Overview

This opensource project 'C2LoRaGW' connects LoRa (SemTech SX126x) based radios with internet / ham-net based repeater software.
To transport speech data using low bandwidth / bitrates C2LoRaGW uses only Codec2 within the UDP-RTP datastream.

The datastream used in the internet / ham-net derives from a 2012 project 'HAMdLNK' and uses the same format like my D-Star
proof-of-concept software from this time. A demo software 'DV-RPTR-App' was already developed and is still in use (now with
localaudio using pipewire) to test the UDP link.

On UHF side I defined 10 modes using the lower bandwidth modulations of the LoRa IC: 10,15,20 and 31kHz to transmit with 100%
duty on a frequency within 70cm ham-radio band (430-440MHz in Germany). C2LoRaGW acts as a hotspot transmitting and receiving on
the same frequency half-duplex. There is an option to add a 2nd transceiver to it and build a full duplex relay base station.

Because the fact that not ready-to-use LoRa based radios exist, this project contains also audio in- and output using cheap I2S
modules plus display an simple UI on a TFT or OLED display.

Project description [here](documentation/project_description.md)

### Hardware

A ready to use 80x50mm PCB *"C2LORAGW"* containing an E22-400M30S (or M33S) and a ESP32-S3 module is available. All spare IO's are
routed to a 2.54mm pin header for adding extensions. This board can be powered by an USB 2A charger or an LiPo / LiIon 1S battery.
To simplify the design this board doesn't contain a charger or power switch (can be added on a separate board).

This PCB is sized to fit into an Hammond or Fisher aluminum case, but also have 4 mounting holes (M3).

Detailed hardware description [here](documentation/hardware.md)

### Firmware

Because the project is based around the ESP32-S3 (only this µC) the firmware needs only the EspressIf IDF developing environment.
As IDE there are some options, I used Microsoft VS Code. Setup like on EspressIf documentation described. This project uses
other opensource: Codec2 for example and the SemTech sx126x driver.

## Getting Started

First: READ all available documentation (it is not much). See "documentation" folder.

Get some compatible hardware (currently supported off-the-shelf products: C2LORAGW, LiliGo T_Deck^*) or build some.

(*) = my T-Deck still not work (Lora32 module fails).

### Flash Precompiled Firmware

1. Download the [ESP32 flashing utility](https://github.com/cpq/esputil).
2. Download the precompiled firmware image (ZIP file with a few binary images)
3. Flash the stuff

tbd

### First Connection

Run the EspressIf monitor or something similar (Putty) to see C2LORAGW boot. After this, it shows a command prompt. Using typed
commands to establish a wifi connection (station mode) or change C2LORA settings, hostname, callsign and so on.

### Calibration

Because of the narrow bandwidth used a frequency calibration of crystal based LoRa modules is mandatory. The frequency needs to
be within a range of +/- 2.6kHz of the target value. TCXO radio modules are much better and a calibration is not required.

Calibration requires a frequency counter or an similar device to view a carrier in the 70cm band. Start calibration using the
command line:

`c2lora cali 1`

After this command the device transmits a continuous wave (unmodulated carrier). On the terminal the actual used offset value is
shown. Measure the difference between the carrier and the target frequency and add or subtract it from the shown offset. Type
in the new calculated offset:

`c2lora offset <new value>`

After this, the carrier moves to a new position.

**Warning:** Use an antenna or an dummy load while doing this!

A rule of thumb: Cheap crystal based modules have shown an initial offset of +17kHz below the target.

If done, turn of the transmitter with:

`c2lora cali 0`

### Save C2LORA Settings

All settings (frequency, mode, txpower and offset) are saved with the command

`c2lora save`


## Other And Custom Targets

### Build Hardware

Get an ESP32-S3 wroom based dev-kit or similar. Connect a SX1262 or SX1268 based Lora module (equipped for UHF!) to it. There is
almost no constrains in pinning the SPI or the additional GPIO pins needed.

### Get Repository

* Install VS code with the EspressIf toolchain + esp-idf (or just the bare EspreeIf stuff)
* Clone the repository from here: https://github.com/DO1FJN/C2LORAGW
* copy one of the provided sdkconfig to "sdkconfig"
* Modify the GPIO and other settings in ./components/boards/breadboard.h or create a new board definition file for your target
* Config the project...  `idf.py menuconfig` (select the proper console for your target)
* Build + flash it... `idf.py flash`

tdb

### LilyGo T-Deck

My T-Deck LORA32 module doesn't work (XOSC startup error), so i stalled the development for this for now.

tbd
