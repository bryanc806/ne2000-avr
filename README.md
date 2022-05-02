# ne2000-avr
LUFA based RNDIS USB adapter to control a 16-bit NE2000 Ethernet card

This is a silly project to create a poor USB to Ethernet adapter.

## Construction

I built mine on a Teensy++ 2.0.  You will need a microcontroller that is at least 5V tolerant.  I just wire-wrapped and soldered my prototype.  Nothing fancy.

Connections:

| ISA Bus Signal | AVR Signal |
|----------------|------------|
| D0-D7          | Port C (note that on a teensy++ 2.0, port C and E[0,1] align with D0-D7 IOCHRDY and AEN, so you can attach them directly |
| D8 - D16       | Port A     |
| A0 - A7        | Port D     |
| A8 - A11       | Port B [0:3] |
| /IOR           | B[4]       |
| RESET          | B[5]       |
| /IOW           | B[6]       |
| CLK            | B[7]       |
| AEN            | E[0]       |
| IOCHRDY        | E[1]       |
| ALE            | E[6]       |
| IOCS16         | E[7]       |


Port F is use to drive LEDs for debugging and showing TX/RX status.

## Compiling

 - Get the lufa library from github
 - Edit the makefile in this repo to point to the LUFA directory in the lufa repo.
 - type make

## Performance

Currently, I can get about 2Mb/sec RX performance and 2.5 Mb/sec TX.  This is slower than it should be; still optimizing.
