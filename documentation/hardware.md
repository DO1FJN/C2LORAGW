# C2LoRaGW Hardware

## Prototyp 1 Board (PT01)

### Issues

ZRXE125 polarity wrong. Part needs to be "upside down" or 1180° rotated. An additional capacitor is placed on R11 to
prevent fast switch on/off at the exact trigger voltage.

The 3.3VCC supply drops below APX809S31SA trigger level, if a sudden load happen on 5V rail (turning it on or transmit)
only if the input voltage is in the range between 3.8V - 3.3V (some mV above 3.3V output).

## Full-Duplex Relay-Addon (FDR01)

tbd
