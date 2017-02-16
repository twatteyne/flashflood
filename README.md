## reference power consumption

| Project             | explanation         | raw results                                                                                                        | average current |
| ------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------- |
| `ref_radiorx`       | radio on rx         | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256830000&to=1487256900000) |         23-24mA |
| `ref_radioosc`      | radio oscillator on | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256945000&to=1487257020000) |         23-24mA |
| `ref_radiooff_lpm0` | radio off, LPM0     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256340000&to=1487256410000) |          1.90mA |
| `ref_radiooff_lpm3` | radio off, LPM3     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256510000&to=1487256590000) |          1.80mA |
| `ref_radiooff_lpm4` | radio off, LPM4     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256650000&to=1487256730000) |          1.75mA |

## Timer A

32 kHz (ACLK), continuous mode

| interrupt      | brief                         | trigger time             | interrupt routine | comment |
|----------------|-------------------------------|--------------------------|-------------------|---------|
| `CCR1` compare | calculating subticks          | every 224 ticks (6836us) | read the TB register first, then calculate the difference (offset) to the TB value recorded at last interrupt. Subticks = offset<<5||
| `CCR2` compare | sampling light sensor         | every 100 ticks (3050us) | detect whether light changed its status, if so, send a data packet | only for sensing node |
| `overflow`     | do nothing                    | every 65536 ticks        | none              | |

## Timer B

5 MHz (SMCLK), continuous mode

| interrupt        | brief                             | trigger time                                         | interrupt routine | comment |
|------------------|-----------------------------------|------------------------------------------------------|-------------------|---------|
| `CCR1` capture   | capture on radio SFD pin          | rising or falling edge of SFD pin                    | first read the length, FCF and DSN field of the frame, if this is ACK and the DSN is newer than me, re-transmit data packet | only call on end of received frame |
| `CCR2` compare   | execute data transmission         | subticks (around 213.5us) after end of receiving ACK | send TXON strobe for sending data | |
| `overflow`       | do nothing                        | every 65536 ticks                                    | none              | |


## pins

| pin  | when high                           | when low							| when toggle |
|------|-------------------------------------|-----------------------------------|-------------|
| P2.3 | light is on                         |  light is off                     ||
| P6.6 | interrupt routine of timer B starts |  interrupt routine of timer B end ||
| P6.7 | start of sending TXON strobe        |  TXON strobe senddone             ||
| P3.4 | interrupt routine of timer A starts |  interrupt routine of timer A end ||
| P2.6 |                                     |                                   | `CCR1` interrupt of timer A trigger |
| P3.5 |                                     |                                   | I need re-transmit later            |