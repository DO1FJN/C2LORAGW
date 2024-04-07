# C2LORA Transmission Modes

The proper transmission mode must be known in advance. Like the frequency it is set once. There are 10 modes defined.

## Change Mode

Open a terminal / monitor to the board. Type

`c2lora mode x`

and hit enter.

(replace the 'x' with the mode number 0 to 9 or with the exact mode name (like "15kHz_1600").

## Mode Details

All modes have in common a 480ms long packet that have the same amount of Codec2 digital speech in it. There are almost no gaps
between packets. The transmitter sends out a stream of packet continuously (100% duty).

The first packet starts with a header containing some information:
* callsign (max 7 characters)
* recipient (max 7 characters, specials like "CQCQCQ" possible)
* kind of station (kos) and "was repeated" bit
* a 16bit crc for the values above
* (not in all modes) area code
* (not in all modes) locator

If configured with a frequency shift, the downstream (higher frequency, usually transmitted by a relay station / repeater) uses
inverted IQ for better separation. The upstream still uses normal IQ, same like in single frequency mode.

For all modes the SYNC WORD 1464h is used.

## Overview

| mode      | name | bandwidth | SF | CR  | Codec2 | RX sensitivity | eff. bitrate |
| --------- | ---- | :-------: | -- | --  | -----: | -------------: | -----------: |
|C2M0_10LR  | 10kHz_700C | 10kHz | SF5 | 4/7 | 700C | -129.8dBm |  930bps |
|C2M1_15LR  | 15kHz_700C | 15kHz | SF6 | 4/6 | 700C | -130.6dBm |  977bps |
|C2M2_15STD | 15kHz_1300 | 15kHz | SF5 | 4/6 | 1300 | -128.1dBm | 1628bps |
|C2M3_15MQ  | 15kHz_1400 | 15kHz | SF5 | 4/6 | 1400 | -128.1dBm | 1628bps |
|C2M4_15HQ  | 15kHz_1600 | 15kHz | SF5 | 4/5 | 1600 | -128.1dBm | 1953bps |
|C2M5_20LR  | 20kHz_1300 | 20kHz | SF6 | 4/5 | 1300 | -129.3dBm | 1524bps |
|C2M6_20    | 20kHz_2400 | 20kHz | SF5 | 4/5 | 2400 | -126.8dBm | 2604bps |
|C2M7_31STD | 31kHz_1600 | 31kHz | SF6 | 4/6 | 1600 | -127.6dBm | 1953bps |
|C2M8_31LL  | 31kHz_2400 | 31kHz | SF5 | 4/7 | 2400 | -125.1dBm | 3255bps |
|C2M9_31HQ  | 31kHz_3200 | 31kHz | SF5 | 4/5 | 3200 | -125.1dBm | 3906bps |

* bandwidth (BW) refers to the double sideband (DSB)
* spreading factor (SF): only the lowest two SF are used, not compatible with SX127X devices
* FEC coding rate (CR): forward error detection + correction overhead (4/5 means 4 data bits encoded with 5 bits on air)
* effective bitrate: calculated raw bitrate over the air

## Details

### C2M0_10LR "long range"

Having only a 12.5kHz wide channel to operate, this is the only mode left. A SX126x IC can transmit even with a narrower
bandwidth of 7.8kHz, but there is no benefit to use it.

With a BW of 10.4kHz and SF5 there is room to a increased CR for better single bit error resistance . The audio quality of
Codec2 in 700C mode is acceptable.

* minimum preamble length 12 = 36.86ms
* default preamble length 15 = 46.08ms, first packet (with header) 14
* payload size = 47 bytes (1 frame start byte, 42 bytes speech data, 4 bytes for cyclic data)
* header size = 13 bytes (no area code, no locator)
* first packet onair time = 580ms

### C2M1_15LR "long range"

A 15.6kHz wide signal is used to operate in a 25kHz wide channel. Using SF6 results in longer symbol times and in a more
resilient signal at the cost of bitrate. The purpose of C2M1_15LR is to get really far out.

* default preamble length 12 = 49.15ms, first packet (with header) 14
* payload size = 48 bytes (1 frame start byte, 42 bytes speech data, 5 bytes for cyclic data)
* header size = 13 bytes (no area code, no locator)
* first packet onair time = 580ms

### C2M2_15STD "universal"

With a slightly better CR and more room for extra data this mode is versatile but still lacks in additional header information.

* minimum preamble length 12 = 24.58ms
* default preamble length 15 = 30.72ms, first packet (with header) 20
* payload size = 87 bytes (1 frame start byte, 78 bytes speech data, 8 bytes for cyclic data)
* header size = 15 bytes (with area code, no locator)
* first packet onair time = 560ms

### C2M3_15MQ "better speech"

Just like *C2M2_15STD*, but with Codec2 1400 bits per second and less additional data (slower cyclic data rate).

* minimum preamble length 12 = 24.58ms
* default preamble length 15 = 30.72ms, first packet (with header) 20
* payload size = 87 bytes (1 frame start byte, 84 bytes speech data, 2 bytes for cyclic data)
* header size = 15 bytes (with area code, no locator)
* first packet onair time = 560ms

### C2M4_15HQ "even better speech quality"

Utilizing only a CR of 4/5 gives room for more speech data in exchange for robustness.

* minimum preamble length 12 = 24.58ms
* default preamble length 14 = 28.76ms, first packet (with header) 20
* payload size = 105 bytes (1 frame start byte, 96 bytes speech data, 8 bytes for cyclic data)
* header size = 15 bytes (with area code, no locator)
* first packet onair time = 550ms

### C2M5_20LR "better quality long range"

A wider bandwidth of 20.8kHz is used in combination with SF6. So a Codec2 1300 stream will fit event with a good receiver
sensitivity. Only with the base CR of 4/5 possible.

* minimum preamble length 12 = 36.86ms (recommended by SemTech)
* default preamble length 11 = 33.79ms, first packet (with header) 13
* payload size = 81 bytes (1 frame start byte, 78 bytes speech data, 2 bytes for cyclic data)
* header size = 15 bytes (with area code, no locator)
* first packet onair time = 560ms

### C2M6_20 "speech data only"

With this mode a high quality Codec2 stream (2400) is used to get a lower latency even within a 20.8kHz wide transmission. The
latency aspect can improve repeated connection (HAMdLNK) but because of the sampling + encoding time (~32ms) there is no full
effect on a local transmission.

* minimum preamble length 12 = 18.43ms
* default preamble length 12 = 18.43ms, first packet (with header) 13
* payload size = 145 bytes (1 frame start byte, 144 bytes speech data, nothing left)
* header size = 15 bytes (with area code, no locator)
* first packet onair time = 525ms

### C2M7_31STD "widband standard"

Having room for a 31.2kHz wide transmission this mode utilizes SF6 to transport Codec2 1600.

* minimum preamble length 12 = 24.58ms
* default preamble length 16 = 32.77ms, first packet (with header) 13
* payload size = 105 bytes (1 frame start byte, 96 bytes speech data, 8 byte extra)
* header size = 21 bytes (with area code and locator)
* first packet onair time = 560ms

### C2M8_31LL "low latency"

* minimum preamble length 12 = 12.29ms
* default preamble length 12 = 12.29ms, first packet (with header) 20
* payload size = 160 bytes (1 frame start byte, 144 bytes speech data, 15 byte extra)
* header size = 15 bytes (with area code, no locator)
* first packet onair time = 530ms

### C2M9_31HQ "high quality"

At the cost of bandwidth, FEC and RX sensitivity this it the mode to transport Codec2 at the highest quality possible.

* minimum preamble length 12 = 12.29ms
* default preamble length 13 = 13.31ms, first packet (with header) 14
* payload size = 222 bytes (1 frame start byte, 192 bytes speech data, 29 byte extra)
* header size = 20 bytes (with area code and locator)
* first packet onair time = 520ms
