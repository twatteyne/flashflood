* schedule: http://wsn-testbed.it.uu.se:5000/
* real-time Grafana: http://wsn-testbed.it.uu.se:3000/dashboard/db/experiment-overview?from=now-1m&to=now

## DSN contents

```
  7 6 5 4 3 2 1 0
 +-+-+-+-+-+-+-+-+
 |L| hop |  seq  |
 +-+-+-+-+-+-+-+-+
```

* `L`: state light (0=off, 1=high)
* `seq`: increments at each new light switch
* `hop`: sensing node is hop 0

## reference power consumption

| Project             | explanation         | raw results                                                                                                        | average current |
| ------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------- |
| `ref_radiorx`       | radio on rx         | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256830000&to=1487256900000) |         23-24mA |
| `ref_radioosc`      | radio oscillator on | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256945000&to=1487257020000) |         23-24mA |
| `ref_radiooff_lpm0` | radio off, LPM0     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256340000&to=1487256410000) |          1.90mA |
| `ref_radiooff_lpm3` | radio off, LPM3     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256510000&to=1487256590000) |          1.80mA |
| `ref_radiooff_lpm4` | radio off, LPM4     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256650000&to=1487256730000) |          1.75mA |

## Timer A

* 32 kHz (ACLK), continuous mode
* never stopped, rolls over ever 2s

| interrupt      | description |
|----------------|-------------|
| `CCR1` compare | Used to calibrate ~5 MHz clock speed: how many subticks in 7 ticks?. Fires 32*7 ticks (6836us), compare TBR against last value of TBR. Then `value<<5` is the number of subticks in 7 ticks |
| `CCR2` compare | Used to trigger light sensor sampling. Fires every 100 ticks (~3050us) |
| `overflow`     | _enabled by default, but no action_ |

## Timer B

* 5 MHz (SMCLK), continuous mode
* never stopped (TODO: needs fixing)

| interrupt        | description |
|------------------|-------------|
| `CCR1` capture   | Triggered by the SFD pin toggling, triggers an interrupt. Do nothing on rising edge (start of frame). On falling edge (end of frame), read packet and schedule data transmission, if appropriate. |
| `CCR2` compare   | Fires when it's time to forward the packet, around 213.5us after end of frame of the ACK |
| `overflow`       | _enabled by default, but no action_ |

## pins

| pin  | active when   | description |
|------|---------------|-------------|
| P2.3 |               | output pin on the sink node (possibly on all nodes) |
| P3.4 | `LOCAL_SETUP` | high during Timer A ISR |
| P6.6 | `LOCAL_SETUP` | high during Timer B ISR |
| P2.6 | `LOCAL_SETUP` | toggle at beginning of calibration |
| P3.5 | `LOCAL_SETUP` | toggle when mote decides to retransmit (after ACK) |
| P6.7 | `LOCAL_SETUP` | TODO |
