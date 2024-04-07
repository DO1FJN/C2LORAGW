# C2LoRa

## What Is C2LoRa?

C2LoRa stands for "Codec2 over LoRa". It utilizes a SemTech SX126x IC (SX1261, SX1262 or SX1268) as a transceiver for
digital speech data. This digital speech data is heavily compressed by an opensource vocoder "Codec2".

## What Is Codec2

Codec2 is a vocoder. Developed by David Rowe, Codec2 is a open source encoder and decoder specialized for human speech.
This algorithm is used to get speech data with only 3.2 down to 0.7 kbit/s (normal phone quality PCM audio: 64kbit/s).

## What Is HAMdLNK

HAMdLNK (ham radio digital voice link) is a protocol definition that adds a subframe definition to a UDP-RTP packet. It's a way
to uses UDP-RTP and pack some more information into it beside the speech data like sender, recipient and much more.

## What Is The 'Gateway'

C2 LoRa Gateway - the name of this project - describes the main function: A C2LoRa reception is forwarded to an IP address and
incoming HAMdLNK packets will be assembled and transmitted. The forwarded IP address can configured as an IPv4 broadcast or IPv6
multicast or as a URL (DNS resolvable name). IP base communication is done using the integrated

In the simplest scenario 2 devices are within the same local LAN. One uses a Gateway configuration, the other is in a 'dongle'
mode. With the 'dongle' device an operator can remotely receive and transmit away from the main station (roof antenna).

## How It Works

C2LoRa defines 10 slightly different modes. All modes in common are the use of a narrowish bandwidth (10-31kHz) and a very low
spreading factor (SF5 o SF6). For this reason the modulation isn't compatible with older SX127x IC's. In all modes speech will
be framed into 480ms long packets (6 or 12 Codec2 frames). These packets are transmitted using the mode-depended parameters. On
air such a packet is roughly as long as the voice (480ms) so that only minimal gaps exists and a transmission is near 100% duty.

The first packet of every transmission contains a tiny header so that the first transmission is between 520-580ms long.

Here is the mode overview: [C2LORA modes](C2LORA modes.md)

## Hardware Setup

This project utilizes EspressIF IDF toolchain and will compile only on a ESP32-S3 (wroom module) in the moment. This SOC is
fast enough to handle the Codec2 en- and decoding. Besides the SOC there is an module with an SX126x IC needed. In my design
it's a EByte E22-400M30S. This model is equipped with a TCXO so that the frequency is stable and even w/o calibration usable.
Module w/o a TCXO need a calibration because the frequency is very off (I measured 17kHz for my testing modules). Because of
the narrow bandwidth the modules need to be calibrated (max 2.6kHz off). Beside the power regulator(s) and a USB-C plug plus 2
buttons: That's really is it.

### Optional Hardware

There is "no limit". In the moment I added support for the common MAX98357A I2S audio amplifier and an INMP441 microphone. The
BOOT key acts as a PTT. A ST7789 TFT display can be also used. I plan to put more stuff (touchscreen support) into in so that
for example a LilyGo T-Deck can be used as a handheld radio.

### Developed Hardware

"C2LORAGW" description [here](hardware.md)
