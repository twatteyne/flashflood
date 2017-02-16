## reference power consumption

| Project             | explanation         | raw results                                                                                                        | average current |
| ------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------- |
| `ref_radiorx`       | radio on rx         | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256830000&to=1487256900000) |         23-24mA |
| `ref_radioosc`      | radio oscillator on | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256945000&to=1487257020000) |         23-24mA |
| `ref_radiooff_lpm0` | radio off, LPM0     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256340000&to=1487256410000) |          1.90mA |
| `ref_radiooff_lpm3` | radio off, LPM3     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256510000&to=1487256590000) |          1.80mA |
| `ref_radiooff_lpm4` | radio off, LPM4     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256650000&to=1487256730000) |          1.75mA |

## Related Timers ##

| timers    | freqency(Hz) | clockSource | mode         | interrupt                        |
|-----------|--------------|-------------|--------------|----------------------------------|
| `timer A` | 32768        | ACLK        | continueMode | 2 compare, 1 overflow            |
| `timer B` | around 5M    | SMCLK       | continueMode | 1 compare, 1 overflow, 1 capture |


### Timer A's interrupts ###

| interrupt | brief                         | trigger time             | interrupt routine | comment |
|-----------|-------------------------------|--------------------------|-------------------|---------|
| `CCR1`    | calculating subticks          | every 224 ticks (6836us) | read the TB register first, then calculate the difference (offset) to the TB value recorded at last interrupt. Subticks = offset<<5||
| `CCR2`    | sampling light sensor         | every 100 ticks (3050us) | detect whether light changed its status, if so, send a data packet | only for sensing node |
| `overflow`| do nothing                    | every 65536 ticks        | none              | |

### Timer B's interrupts ###

| interrupt | brief                             | trigger time                                         | interrupt routine | comment |
|-----------|-----------------------------------|------------------------------------------------------|-------------------|---------|
| `CCR1`    | capture on radio SFD pin          | rising or falling edge of SFD pin                    | first read the length, FCF and DSN field of the frame, if this is ACK and the DSN is newer than me, re-transmit data packet | only call on end of received frame |
| `CCR2`    | execute data transmission         | subticks (around 213.5us) after end of receiving ACK | send TXON strobe for sending data | |
| `overflow`| do nothing                        | every 65536 ticks                                    | none              | |
