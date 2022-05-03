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

 - Get the lufa library from https://github.com/abcminiuser/lufa
 - Edit the makefile in this repo to point to the LUFA directory in the lufa repo.
 - type make

## Performance

Currently, I can get about 2.90Mb/sec RX performance and 2.36 Mb/sec TX:

```
$ iperf3 -c xxx.xxx.xxx.xx
Connecting to host xxx.xxx.xxx.xxx, port 5201
[  5] local yyy.yyy.yyy.yyy port 38628 connected to xxx.xxx.xxx.xxx port 5201
[ ID] Interval           Transfer     Bitrate         Retr  Cwnd
[  5]   0.00-1.00   sec   415 KBytes  3.40 Mbits/sec    0   28.0 KBytes       
[  5]   1.00-2.00   sec   246 KBytes  2.02 Mbits/sec    0   28.0 KBytes       
[  5]   2.00-3.00   sec   308 KBytes  2.52 Mbits/sec    6   12.6 KBytes       
[  5]   3.00-4.00   sec   246 KBytes  2.02 Mbits/sec    0   12.6 KBytes       
[  5]   4.00-5.00   sec   308 KBytes  2.52 Mbits/sec    4   9.80 KBytes       
[  5]   5.00-6.00   sec   246 KBytes  2.02 Mbits/sec    5   11.2 KBytes       
[  5]   6.00-7.00   sec   308 KBytes  2.52 Mbits/sec    0   11.2 KBytes       
[  5]   7.00-8.00   sec   246 KBytes  2.02 Mbits/sec    2   11.2 KBytes       
[  5]   8.00-9.00   sec   246 KBytes  2.02 Mbits/sec    0   11.2 KBytes       
[  5]   9.00-10.00  sec   308 KBytes  2.52 Mbits/sec    2   8.40 KBytes       
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID] Interval           Transfer     Bitrate         Retr
[  5]   0.00-10.00  sec  2.81 MBytes  2.36 Mbits/sec   19             sender
[  5]   0.00-10.02  sec  2.66 MBytes  2.23 Mbits/sec                  receiver

iperf Done.
$ iperf3 -c xxx.xxx.xxx.xxx -R
Connecting to host xxx.xxx.xxx.xxx, port 5201
Reverse mode, remote host xxx.xxx.xxx.xxx is sending
[  5] local yyy.yyy.yyy.yyy port 38632 connected to xxx.xxx.xxx.xxx port 5201
[ ID] Interval           Transfer     Bitrate
[  5]   0.00-1.00   sec   343 KBytes  2.81 Mbits/sec                  
[  5]   1.00-2.00   sec   347 KBytes  2.85 Mbits/sec                  
[  5]   2.00-3.00   sec   346 KBytes  2.83 Mbits/sec                  
[  5]   3.00-4.00   sec   347 KBytes  2.85 Mbits/sec                  
[  5]   4.00-5.00   sec   343 KBytes  2.81 Mbits/sec                  
[  5]   5.00-6.00   sec   351 KBytes  2.88 Mbits/sec                  
[  5]   6.00-7.00   sec   329 KBytes  2.70 Mbits/sec                  
[  5]   7.00-8.00   sec   363 KBytes  2.97 Mbits/sec                  
[  5]   8.00-9.00   sec   351 KBytes  2.88 Mbits/sec                  
[  5]   9.00-10.00  sec   328 KBytes  2.68 Mbits/sec                  
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID] Interval           Transfer     Bitrate         Retr
[  5]   0.00-10.01  sec  3.46 MBytes  2.90 Mbits/sec   75             sender
[  5]   0.00-10.00  sec  3.37 MBytes  2.83 Mbits/sec                  receiver

iperf Done.
```

There's probably room for optimization.  The ISA bus timing is approximated at best and probably somewhat limited by the choice of I/O pins.  With that stated, I think much of the time is spent doing usb things.


